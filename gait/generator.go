package gait

import (
	"encoding/json"
	"math"
)

// Mode 标记生成器当前所处的调度状态。
type Mode int

const (
	Running  Mode = iota // 正常按相位表摆腿/支撑
	Dwelling             // 切换过渡：四足着地驻留，速度回零，等待相位表切换
)

func (m Mode) String() string {
	if m == Dwelling {
		return "dwell"
	}
	return "run"
}

// FootSnapshot 是某一时刻单只脚的位置/速度采样。
type FootSnapshot struct {
	X, Y, Z    float64
	VX, VY, VZ float64
	Swing      bool
}

// Frame 是导出的一帧仿真状态（供可视化与检测使用）。
type Frame struct {
	T     float64 `json:"t"`
	Gait  string  `json:"gait"`
	Mode  string  `json:"mode"`
	Phase float64 `json:"phase"`
	BodyX float64 `json:"bx"`
	V     float64 `json:"v"`
	ComX  float64 `json:"cx"`
	ComY  float64 `json:"cy"`
	Legs  [4]FootSnapshot

	Stable       bool    `json:"stable"`
	Margin       float64 `json:"margin"` // NaN 时序列化为 null
	SupportCount int     `json:"nSup"`
	Reason       string  `json:"reason,omitempty"`
	Polygon      []XY    `json:"poly,omitempty"`
	Switch       bool    `json:"switch,omitempty"`
}

// TouchdownEvent 记录一次落脚事件，供落脚速度收敛性检测。
type TouchdownEvent struct {
	Leg  int     `json:"leg"`
	T    float64 `json:"t"`
	Vz   float64 `json:"vz"`   // 落脚前一瞬的垂直速度
	VXY  float64 `json:"vxy"`  // 落脚前一瞬的水平速度
	Jump float64 `json:"jump"` // 落脚瞬间两侧位置之差（应为 0）
}

// SwitchEvent 记录一次步态切换：请求时刻与实际切换时刻。
type SwitchEvent struct {
	RequestT    float64 `json:"requestT"`
	SwitchT     float64 `json:"switchT"`
	From        string  `json:"from"`
	To          string  `json:"to"`
	AllGrounded bool    `json:"allGrounded"`
	MaxJump     float64 `json:"maxJump"` // 切换瞬间各足位置跳变（应为 0）
}

// legRuntime 是生成器内部的单腿运行状态。
type legRuntime struct {
	inSwing              bool
	anchor               Vec3 // 支撑相落脚点（世界系，Z=0）
	p0, p1               Vec3 // 当前/最近一次摆动的起止点
	swingStart, swingDur float64
	nextLift             float64 // 支撑相下一次抬脚的绝对时刻；+Inf 表示被挂起切换抑制
}

type velRamp struct {
	t0, t1, v0, v1 float64
}

type pendingSwitch struct {
	requestT float64
	params   Params
	targetV  float64
	active   bool // 是否已到请求时刻（请求可提前登记，到点才影响调度）
}

// switchActive 报告切换请求是否已生效。
func (s *Simulator) switchActive() bool { return s.pending != nil && s.pending.active }

// Simulator 是事件驱动的四足步态生成器。
type Simulator struct {
	geo        Geometry
	t          float64
	x          float64 // 机身质心 X（世界系）
	v          float64
	ramp       *velRamp
	params     Params
	cycleStart float64 // 当前步态周期起点（供相位显示）
	mode       Mode
	dwellEnd   float64
	liftH      float64

	legs [4]legRuntime

	pending        *pendingSwitch
	phaseCompleted bool // 挂起切换后，是否已有至少一条腿完成其当前相
	frozenPhase    float64
	comX, comY     float64
	comTargetY     float64   // 当前摆动相内锁定的横向预置目标（世界系 Y）
	comRamp        *comYRamp // 四足支撑窗口内的横向质心穿越计划

	touchdowns []TouchdownEvent
	switches   []SwitchEvent
}

// comYRamp 是四足支撑窗口内质心横向线性穿越的时间表。
type comYRamp struct {
	t0, t1, y0, y1 float64
}

// 过渡与重心调整参数。
const (
	decelLimit        = 0.8  // 驻留过渡减速度 m/s²
	dwellHoldTime     = 0.10 // 速度回零后的四足驻留保持时间 s
	comLatGain        = 1.0 / 3.0
	defaultLiftHeight = 0.06 // 默认抬脚高度 6 cm
	eventEps          = 1e-9
)

// NewSimulator 创建生成器。p 为初始步态参数，v0 为初始机身速度。
func NewSimulator(geo Geometry, p Params, v0 float64) *Simulator {
	s := &Simulator{
		geo: geo, params: p, v: v0, mode: Running,
		liftH: defaultLiftHeight, dwellEnd: math.Inf(1),
		comX: 0, comY: 0,
	}
	for i := LegID(0); i < 4; i++ {
		hip := geo.HipPosition(i)
		s.legs[i] = legRuntime{
			anchor:   Vec3{X: hip.X, Y: hip.Y},
			nextLift: math.Inf(1),
		}
	}
	// 按相位表初始化调度：偏移为 0 的腿在 t=0 立即起步，其余定时抬脚。
	s.scheduleFromTable(0, true)
	// 爬行起步：质心已预置到首批摆动脚的对侧（先移重心、再抬脚）。
	s.comTargetY = s.currentSwingTargetY()
	s.comY = s.comTargetY
	return s
}

// Touchdowns 返回迄今全部落脚事件。
func (s *Simulator) Touchdowns() []TouchdownEvent { return s.touchdowns }

// Switches 返回迄今全部步态切换事件。
func (s *Simulator) Switches() []SwitchEvent { return s.switches }

// Params 返回当前生效的步态参数。
func (s *Simulator) Params() Params { return s.params }

// scheduleFromTable 按当前相位表安排各腿的下一次抬脚。
// immediate=true 时偏移为 0 的腿立刻开始摆动（用于周期 0 起点与切换后）。
func (s *Simulator) scheduleFromTable(t float64, immediate bool) {
	for i := LegID(0); i < 4; i++ {
		l := &s.legs[i]
		o := s.params.offset(i)
		switch {
		case o == 0 && immediate:
			s.beginSwing(i, t)
		case !l.inSwing:
			l.nextLift = t + o/s.params.Frequency
		}
	}
}

// velAt 返回任意时刻的期望速度（线性加减速斜坡 + 常速段）。
func (s *Simulator) velAt(t float64) float64 {
	if s.ramp != nil && t > s.ramp.t0 && t < s.ramp.t1 {
		r := s.ramp
		return r.v0 + (r.v1-r.v0)*(t-r.t0)/(r.t1-r.t0)
	}
	if s.ramp != nil && t >= s.ramp.t1 {
		return s.ramp.v1
	}
	return s.v
}

// RampVelocity 登记一条 [t0,t1] 内 v0->v1 的速度斜坡。
func (s *Simulator) RampVelocity(t0, t1, v1 float64) {
	s.ramp = &velRamp{t0: t0, t1: t1, v0: s.velAt(t0), v1: v1}
}

// integrateVel 解析积分 [from,to) 区间内的速度，得到机身位移。
func (s *Simulator) integrateVel(from, to float64) float64 {
	if to <= from {
		return 0
	}
	if s.ramp == nil {
		return s.v * (to - from)
	}
	r := s.ramp
	v := func(t float64) float64 {
		switch {
		case t <= r.t0:
			return r.v0
		case t >= r.t1:
			return r.v1
		default:
			return r.v0 + (r.v1-r.v0)*(t-r.t0)/(r.t1-r.t0)
		}
	}
	// 梯形法分两段（斜坡内被积函数是线性的，梯形解析精确）。
	pts := []float64{from}
	if r.t0 > from && r.t0 < to {
		pts = append(pts, r.t0)
	}
	if r.t1 > from && r.t1 < to {
		pts = append(pts, r.t1)
	}
	pts = append(pts, to)
	total := 0.0
	for k := 0; k < len(pts)-1; k++ {
		total += 0.5 * (v(pts[k]) + v(pts[k+1])) * (pts[k+1] - pts[k])
	}
	return total
}

// hipWorld 返回 t 时刻某腿髋关节的世界系位置。
func (s *Simulator) hipWorld(i LegID, t float64) Vec3 {
	h := s.geo.HipPosition(i)
	return Vec3{X: s.x + s.integrateVel(s.t, t) + h.X, Y: h.Y}
}

func (s *Simulator) beginSwing(i LegID, t float64) {
	l := &s.legs[i]
	if l.inSwing {
		return
	}
	l.inSwing = true
	l.p0 = l.anchor
	l.swingStart = t
	l.swingDur = (1 - s.params.DutyFactor) / s.params.Frequency
	tTd := t + l.swingDur
	// 落地点：相对落脚时刻髋关节向前半个步长。
	// 步长 L = v_td × 支撑相时长 βT，使支撑相内足端相对髋关节
	// 从 +L/2 对称回退到 -L/2（无滑动假设）。
	hip := s.hipWorld(i, tTd)
	step := s.velAt(tTd) * s.params.DutyFactor / s.params.Frequency
	l.p1 = Vec3{X: hip.X + step/2, Y: hip.Y}
	l.nextLift = math.Inf(1)
	// 抬脚瞬间质心必须已在对侧预置点：取消尚未完成的穿越计划并锁定。
	s.comRamp = nil
	s.comTargetY = s.currentSwingTargetY()
	s.comY = s.comTargetY
}

func (s *Simulator) endSwing(i LegID, t float64) {
	l := &s.legs[i]
	// 落脚前一瞬（u→1⁻）的速度由摆线解析公式给出，应趋于 0。
	_, vBefore := SwingFoot(1-1e-12, l.p0, l.p1, s.liftH, l.swingDur)
	posEnd, _ := SwingFoot(1, l.p0, l.p1, s.liftH, l.swingDur)
	jump := math.Sqrt((posEnd.X-l.p1.X)*(posEnd.X-l.p1.X) +
		(posEnd.Y-l.p1.Y)*(posEnd.Y-l.p1.Y) + posEnd.Z*posEnd.Z)
	l.inSwing = false
	l.anchor = l.p1 // 与摆线末端位置严格一致：无位置跳变
	s.touchdowns = append(s.touchdowns, TouchdownEvent{
		Leg: int(i), T: t,
		Vz: vBefore.Z, VXY: math.Hypot(vBefore.X, vBefore.Y), Jump: jump,
	})
	if s.switchActive() {
		// 切换挂起：不安排下一次抬脚，延展支撑，等待全体着地驻留。
		l.nextLift = math.Inf(1)
		s.phaseCompleted = true
		s.tryBeginDwell(t)
	} else {
		l.nextLift = t + s.params.DutyFactor/s.params.Frequency
		// 刚落脚进入四足支撑窗口：若此刻四足着地，安排横向质心
		// 在窗口内穿越到下一只摆动脚要求的对侧预置点。
		if s.allGrounded() {
			s.planCOMRamp(t)
		}
	}
}

// RequestSwitch 登记一次步态切换请求；不会立即生效。
// 生成器保证：所有在摆腿完成当前摆动、且至少一条腿完成当前相之后，
// 才进入四足驻留并替换相位表（见 tryBeginDwell / completeSwitch）。
func (s *Simulator) RequestSwitch(t float64, p Params, targetV float64) {
	s.pending = &pendingSwitch{requestT: t, params: p, targetV: targetV}
	s.phaseCompleted = false
	// 若请求正落在某个相边界上，支撑腿在 nextEvent 处会先报告相完成；
	// 若此刻恰好四足着地（如在驻留入口），直接满足条件。
	s.tryBeginDwell(t)
}

// allGrounded 报告是否四足全部处于支撑相。
func (s *Simulator) allGrounded() bool {
	for i := range s.legs {
		if s.legs[i].inSwing {
			return false
		}
	}
	return true
}

// tryBeginDwell 在"已有腿完成相且四足着地"时进入驻留过渡。
func (s *Simulator) tryBeginDwell(t float64) {
	if s.mode != Running || s.pending == nil {
		return
	}
	if !s.phaseCompleted || !s.allGrounded() {
		return
	}
	s.mode = Dwelling
	s.frozenPhase = wrap01(s.params.Frequency * (t - s.cycleStart))
	v := s.velAt(t)
	tDecel := math.Abs(v) / decelLimit
	if tDecel < 0.02 {
		tDecel = 0
	}
	if tDecel > 0 {
		s.ramp = &velRamp{t0: t, t1: t + tDecel, v0: v, v1: 0}
	} else {
		s.ramp = nil
		s.v = 0
	}
	s.dwellEnd = t + tDecel + dwellHoldTime
	// 驻留期所有按旧相位表排定的抬脚一律失效，避免旧事件时间戳
	// 小于 dwellEnd 导致事件循环空转。
	for i := range s.legs {
		s.legs[i].nextLift = math.Inf(1)
	}
	// 驻留期质心在四足支撑下平滑回到机身中线（此时四足着地，安全）。
	s.comRamp = &comYRamp{t0: t, t1: s.dwellEnd, y0: s.comY, y1: 0}
}

// completeSwitch 在驻留结束、四脚全部着地时真正替换相位表。
func (s *Simulator) completeSwitch(t float64) {
	pend := s.pending
	from := s.params
	pre := s.footPositions(t)

	s.mode = Running
	s.params = pend.params
	s.cycleStart = t
	s.pending = nil
	s.dwellEnd = math.Inf(1)
	s.phaseCompleted = false
	s.ramp = nil
	s.v = 0
	// 切换后用一个周期把速度平滑加到目标值。
	if pend.targetV != 0 {
		s.ramp = &velRamp{t0: t, t1: t + pend.params.Duration(), v0: 0, v1: pend.targetV}
	}
	post := s.footPositions(t)
	maxJump := 0.0
	for i := range pre {
		if j := math.Hypot(post[i].X-pre[i].X, post[i].Y-pre[i].Y); j > maxJump {
			maxJump = j
		}
	}
	s.switches = append(s.switches, SwitchEvent{
		RequestT: pend.requestT, SwitchT: t,
		From: from.String(), To: pend.params.String(),
		AllGrounded: s.allGrounded(), MaxJump: maxJump,
	})
	// 新相位表在相位 0 处的第一批摆腿从各自锚点抬起，位置连续。
	s.scheduleFromTable(t, true)
}

// nextEventTime 返回当前调度下最近的事件时刻。
func (s *Simulator) nextEventTime() float64 {
	tNext := s.dwellEnd
	if s.pending != nil && !s.pending.active {
		tNext = s.pending.requestT // 切换请求到点激活
	}
	for i := range s.legs {
		l := &s.legs[i]
		var te float64
		if l.inSwing {
			te = l.swingStart + l.swingDur
		}
		if !l.inSwing {
			te = l.nextLift
		}
		if te < tNext {
			tNext = te
		}
	}
	return tNext
}

// processEvent 在时刻 t 处理到期事件。
func (s *Simulator) processEvent(t float64) {
	if s.mode == Dwelling {
		if t >= s.dwellEnd-eventEps {
			s.completeSwitch(t)
		}
		return
	}
	// 切换请求到点：先激活，再处理本时刻的腿部事件，
	// 这样恰好在本时刻到期的抬脚会被抑制（延展支撑而不是起摆）。
	if s.pending != nil && !s.pending.active && t >= s.pending.requestT-eventEps {
		s.pending.active = true
		s.tryBeginDwell(t)
	}
	for i := LegID(0); i < 4; i++ {
		l := &s.legs[i]
		switch {
		case l.inSwing && t >= l.swingStart+l.swingDur-eventEps:
			s.endSwing(i, t)
		case !l.inSwing && !s.switchActive() && t >= l.nextLift-eventEps:
			// 无挂起切换，或请求尚未到点：按相位表正常抬脚。
			s.beginSwing(i, t)
		case !l.inSwing && s.switchActive() && t >= l.nextLift-eventEps:
			// 切换请求已到点且到达支撑相边界：本相宣告完成，延展支撑，
			// 绝不抬脚——这正是"等待当前相完成再换表"的约束。
			// 请求时刻未到前（pending 存在但未激活），正常抬脚摆腿。
			l.nextLift = math.Inf(1)
			s.phaseCompleted = true
			s.tryBeginDwell(t)
		}
	}
}

// advanceTo 把生成器推进到目标时刻，期间按时间顺序处理全部事件。
func (s *Simulator) advanceTo(target float64) {
	for s.t < target-eventEps {
		te := s.nextEventTime()
		if te >= target-eventEps {
			s.integrate(target)
			return
		}
		s.integrate(te)
		s.processEvent(te)
	}
}

// integrate 推进连续状态（机身位置、周期相位、质心调整），不处理离散事件。
func (s *Simulator) integrate(to float64) {
	dt := to - s.t
	if dt <= 0 {
		return
	}
	s.x += s.integrateVel(s.t, to)
	if s.ramp != nil && to >= s.ramp.t1 {
		s.v = s.ramp.v1
		s.ramp = nil
	}
	s.t = to
	s.updateCOM(dt)
}

// swingCOMTargetY 返回单只摆动脚要求的横向质心预置位置：
// 把质心移到摆动脚对侧 b/3 处（左脚踏摆时重心右移，反之亦然）。
func (s *Simulator) swingCOMTargetY(swingLeg LegID) float64 {
	hip := s.geo.HipPosition(swingLeg)
	if hip.Y >= 0 {
		return -s.geo.HalfWidth * comLatGain // FL/BL 在左侧 -> 重心右移
	}
	return s.geo.HalfWidth * comLatGain
}

// currentSwingTargetY 对所有在摆脚的预置目标取平均：
// 爬行单脚摆动即其对侧目标；小跑对角成对摆动时左右抵消为 0。
func (s *Simulator) currentSwingTargetY() float64 {
	var sum float64
	n := 0
	for i := range s.legs {
		if s.legs[i].inSwing {
			sum += s.swingCOMTargetY(LegID(i))
			n++
		}
	}
	if n == 0 {
		return 0
	}
	return sum / float64(n)
}

// planCOMRamp 在四足着地的支撑窗口内安排一次横向质心线性穿越，
// 必须在下一只脚抬起之前到达它要求的预置位置。
func (s *Simulator) planCOMRamp(t float64) {
	next := -1
	tMin := math.Inf(1)
	for i := range s.legs {
		l := &s.legs[i]
		if !l.inSwing && l.nextLift < tMin {
			tMin = l.nextLift
			next = i
		}
	}
	var y1 float64
	tEnd := tMin
	if next < 0 {
		y1 = 0 // 无后续抬脚（驻留过渡）：回中
		tEnd = s.dwellEnd
	} else {
		y1 = s.swingCOMTargetY(LegID(next))
	}
	if !(tEnd > t) {
		s.comY = y1
		s.comRamp = nil
		return
	}
	s.comRamp = &comYRamp{t0: t, t1: tEnd, y0: s.comY, y1: y1}
}

// updateCOM 推进质心：纵向随机身中立点（位于所有三足支撑三角形的
// 纵向交集内），横向在四足窗口内按计划线性穿越、在摆动相内锁定。
func (s *Simulator) updateCOM(dt float64) {
	s.comX = s.x
	switch {
	case s.comRamp != nil:
		r := s.comRamp
		tt := (s.t - r.t0) / (r.t1 - r.t0)
		if tt >= 1 {
			s.comY = r.y1
			s.comRamp = nil
		} else {
			s.comY = r.y0 + (r.y1-r.y0)*tt
		}
	default:
		s.comY = s.comTargetY
	}
}

// footPositions 计算时刻 t 四足位置（不推进生成器状态）。
func (s *Simulator) footPositions(t float64) [4]Vec3 {
	var pos [4]Vec3
	for i := LegID(0); i < 4; i++ {
		l := &s.legs[i]
		if l.inSwing {
			u := (t - l.swingStart) / l.swingDur
			pos[i], _ = SwingFoot(u, l.p0, l.p1, s.liftH, l.swingDur)
		} else {
			pos[i] = l.anchor
		}
	}
	return pos
}

func (s *Simulator) footVelocities(t float64) [4]Vec3 {
	var vel [4]Vec3
	for i := LegID(0); i < 4; i++ {
		l := &s.legs[i]
		if l.inSwing {
			u := (t - l.swingStart) / l.swingDur
			_, vel[i] = SwingFoot(u, l.p0, l.p1, s.liftH, l.swingDur)
		}
	}
	return vel
}

func (s *Simulator) phaseAt(t float64) float64 {
	if s.mode == Dwelling {
		return s.frozenPhase // 驻留期相位显示冻结
	}
	return wrap01(s.params.Frequency * (t - s.cycleStart))
}

// Sample 返回时刻 t 的一帧（会把生成器推进到 t），并完成静态稳定性判定。
func (s *Simulator) Sample(t float64) Frame {
	s.advanceTo(t)
	pos := s.footPositions(t)
	vel := s.footVelocities(t)
	f := Frame{
		T: t, Gait: s.params.String(), Mode: s.mode.String(),
		Phase: s.phaseAt(t), BodyX: s.x, V: s.velAt(t),
		ComX: s.comX, ComY: s.comY,
		Margin: math.NaN(),
	}
	supp := make([]XY, 0, 4)
	for i := LegID(0); i < 4; i++ {
		l := &s.legs[i]
		f.Legs[i] = FootSnapshot{
			X: pos[i].X, Y: pos[i].Y, Z: pos[i].Z,
			VX: vel[i].X, VY: vel[i].Y, VZ: vel[i].Z, Swing: l.inSwing,
		}
		if !l.inSwing {
			supp = append(supp, XY{X: l.anchor.X, Y: l.anchor.Y})
		}
	}
	r := StaticStability(XY{X: s.comX, Y: s.comY}, supp)
	f.Stable = r.Stable
	f.Margin = r.Margin
	f.SupportCount = r.SupportCount
	f.Reason = r.Reason
	f.Polygon = r.Polygon
	return f
}

// Run 从当前时刻仿真到 tEnd，按 dt 等距采样并返回全部帧。
// switchFrames 给出发生相位表切换的采样时刻（由切换事件匹配，用于标记）。
func (s *Simulator) Run(tEnd, dt float64) []Frame {
	n := int(math.Ceil(tEnd/dt)) + 1
	frames := make([]Frame, 0, n)
	switched := map[float64]bool{}
	for k := 0; k < n; k++ {
		t := float64(k) * dt
		if t > tEnd+eventEps {
			break
		}
		f := s.Sample(t)
		for _, sw := range s.switches {
			if math.Abs(sw.SwitchT-t) <= dt/2 && !switched[sw.SwitchT] {
				f.Switch = true
				switched[sw.SwitchT] = true
			}
		}
		frames = append(frames, f)
	}
	return frames
}

// MarshalJSON 自定义帧序列化：支撑足不足时裕度为 NaN，输出为 null
// （encoding/json 默认会把 NaN 编码为非法 JSON）。
func (f Frame) MarshalJSON() ([]byte, error) {
	margin := any(nil)
	if !math.IsNaN(f.Margin) {
		margin = f.Margin
	}
	legs := make([]map[string]any, 4)
	for i, l := range f.Legs {
		legs[i] = map[string]any{
			"x": l.X, "y": l.Y, "z": l.Z,
			"vx": l.VX, "vy": l.VY, "vz": l.VZ, "swing": l.Swing,
		}
	}
	poly := any(nil)
	if f.Polygon != nil {
		pp := make([][2]float64, len(f.Polygon))
		for i, p := range f.Polygon {
			pp[i] = [2]float64{p.X, p.Y}
		}
		poly = pp
	}
	out := map[string]any{
		"t": f.T, "gait": f.Gait, "mode": f.Mode, "phase": f.Phase,
		"bx": f.BodyX, "v": f.V, "cx": f.ComX, "cy": f.ComY,
		"legs": legs, "stable": f.Stable, "margin": margin,
		"nSup": f.SupportCount, "reason": f.Reason, "poly": poly,
		"switch": f.Switch,
	}
	return json.Marshal(out)
}
