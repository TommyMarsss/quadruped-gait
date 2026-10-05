package main

import "math"

// SimParams 是机器人本体与仿真的固定参数。
type SimParams struct {
	HipX      float64 `json:"hipX"`      // 髋关节相对机体中心的前向距离 (m)
	HipY      float64 `json:"hipY"`      // 髋关节相对机体中心的侧向距离 (m)
	BodyH     float64 `json:"bodyH"`     // 机身（重心）高度 (m)
	StepH     float64 `json:"stepH"`     // 摆动相抬脚峰值高度 (m)
	MinMargin float64 `json:"minMargin"` // 静态稳定裕度阈值 (m)
}

// legState 是单腿的运行时状态。支撑相中足端在世界系保持不动，
// 摆动相中足端沿摆线从 Liftoff 运动到 Touchdown。
type legState struct {
	Swing     bool
	Foot      Vec3 // 当前足端世界坐标
	Liftoff   Vec3 // 摆动起点
	Touchdown Vec3 // 摆动目标落点
	LiftoffT  float64
	SwingDur  float64
}

// Frame 是一帧回放数据。
type Frame struct {
	T      float64 `json:"t"`
	Gait   string  `json:"gait"`
	Body   Vec3    `json:"body"`
	Feet   [4]Vec3 `json:"feet"`
	Stance [4]bool `json:"stance"`
	Margin float64 `json:"margin"` // 静态稳定裕度（支撑足<3 时为 -Inf，导出时截断为 -1）
	Stable bool    `json:"stable"`
}

// SwitchEvent 记录一次实际发生的步态切换。
type SwitchEvent struct {
	T    float64 `json:"t"`
	From string  `json:"from"`
	To   string  `json:"to"`
}

// Sim 是四足步态仿真器。机体沿世界系运动，支撑足在世界系固定，
// 摆动足沿摆线轨迹运动到规划的落点。
type Sim struct {
	Params SimParams
	Gait   Gait
	VelFn  func(t float64) Vec2 // 期望机体速度（世界系）
	T      float64
	Body   Vec3 // 机体中心（重心）世界坐标

	pending *Gait
	phaseT0 float64 // 当前步态相位表的时间原点
	legs    [4]legState

	Switches []SwitchEvent
}

// NewSim 构造仿真器：机体位于原点、高度 BodyH，四足站立于名义位置。
func NewSim(p SimParams, g Gait, velFn func(t float64) Vec2) *Sim {
	s := &Sim{Params: p, Gait: g, VelFn: velFn, Body: Vec3{Z: p.BodyH}}
	for i := 0; i < 4; i++ {
		n := s.nominal(i)
		s.legs[i].Foot = Vec3{X: n.X, Y: n.Y}
	}
	return s
}

// nominal 返回腿 i 相对机体中心的名义足端位置（水平面）。
func (s *Sim) nominal(leg int) Vec2 {
	x, y := s.Params.HipX, s.Params.HipY
	switch leg {
	case FL:
		return Vec2{x, y}
	case FR:
		return Vec2{x, -y}
	case RL:
		return Vec2{-x, y}
	default: // RR
		return Vec2{-x, -y}
	}
}

// RequestSwitch 请求切换到步态 g。切换不会立即生效：
// 必须等到当前所有摆动腿完成摆动相（触地）、四足均处于支撑相的
// 安全时刻，才替换相位表，保证任何足的轨迹不发生跳变。
func (s *Sim) RequestSwitch(g Gait) { s.pending = &g }

// SwitchPending 报告是否存在尚未生效的切换请求。
func (s *Sim) SwitchPending() bool { return s.pending != nil }

func (s *Sim) stanceNow(leg int) bool {
	return s.Gait.InStance(leg, s.T-s.phaseT0)
}

func (s *Sim) allStance() bool {
	for i := range s.legs {
		if s.legs[i].Swing {
			return false
		}
	}
	return true
}

// beginSwing 在抬腿时刻规划本次摆动：落点 = 预测触地时刻机体位置
// + 名义偏移 + v·T_stance/2（Raibert 式中性落点，使支撑相中足端
// 相对机体对称地从前向后运动）。
// 横向落点放在摆动中心线（y=0）上而不是机体瞬时横向位置：横向重心
// 摆动是机体的周期运动，若跟随它放置足端，支撑多边形会被不同相位
// 落地的足端拉歪，反而吃掉稳定裕度。
func (s *Sim) beginSwing(i int, v Vec2) {
	g := s.Gait
	l := &s.legs[i]
	l.Swing = true
	l.Liftoff = l.Foot
	l.LiftoffT = s.T
	l.SwingDur = (1 - g.Duty) / g.Freq
	stanceDur := g.Duty / g.Freq
	nom := s.nominal(i)
	l.Touchdown = Vec3{
		X: s.Body.X + v.X*l.SwingDur + nom.X + v.X*stanceDur/2,
		Y: nom.Y,
	}
}

// Step 推进一个时间步并返回当前帧。处理顺序保证切换安全：
//  1. 先结算触地事件（摆动相 → 支撑相）；
//  2. 若存在待切换步态且此刻四足均支撑，则替换相位表；
//  3. 再按（可能已更新的）相位表结算抬腿事件；
//  4. 更新摆动足沿摆线的位置。
func (s *Sim) Step(dt float64) Frame {
	v := s.VelFn(s.T)
	s.T += dt
	s.Body.X += v.X * dt
	s.Body.Y += v.Y * dt

	// 1) 触地事件
	touchdown := false
	for i := range s.legs {
		if s.legs[i].Swing && s.stanceNow(i) {
			s.legs[i].Swing = false
			s.legs[i].Foot = s.legs[i].Touchdown
			touchdown = true
		}
	}
	// 2) 步态切换：仅当某只脚刚完成摆动相、且四足全部处于支撑相时
	if s.pending != nil && touchdown && s.allStance() {
		old := s.Gait
		s.Gait = *s.pending
		s.pending = nil
		s.phaseT0 = s.T
		s.Switches = append(s.Switches, SwitchEvent{T: s.T, From: old.Name, To: s.Gait.Name})
	}
	// 3) 抬腿事件
	for i := range s.legs {
		if !s.legs[i].Swing && !s.stanceNow(i) {
			s.beginSwing(i, v)
		}
	}
	// 4) 摆动足位置
	for i := range s.legs {
		if s.legs[i].Swing {
			sg := (s.T - s.legs[i].LiftoffT) / s.legs[i].SwingDur
			if sg > 1 {
				sg = 1
			}
			s.legs[i].Foot = Cycloid(s.legs[i].Liftoff, s.legs[i].Touchdown, s.Params.StepH, sg)
		}
	}
	return s.frame()
}

func (s *Sim) frame() Frame {
	f := Frame{T: s.T, Gait: s.Gait.Name, Body: s.Body}
	var stanceFeet []Vec2
	for i := range s.legs {
		f.Feet[i] = s.legs[i].Foot
		f.Stance[i] = !s.legs[i].Swing
		if f.Stance[i] {
			stanceFeet = append(stanceFeet, Vec2{s.legs[i].Foot.X, s.legs[i].Foot.Y})
		}
	}
	margin, stable := SupportStability(stanceFeet, Vec2{s.Body.X, s.Body.Y}, s.Params.MinMargin)
	f.Margin = margin
	f.Stable = stable
	return f
}

// sanitizeFrames 将 -Inf 裕度（支撑足 <3）截断为 -1 以便 JSON 序列化。
func sanitizeFrames(fs []Frame) {
	for i := range fs {
		if math.IsInf(fs[i].Margin, -1) {
			fs[i].Margin = -1
		}
	}
}
