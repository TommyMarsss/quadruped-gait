package main

import "math"

// Vec2 是水平面（世界系 x 前向、y 左向）上的向量。
type Vec2 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Vec3 是三维向量，z 竖直向上。
type Vec3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

func (a Vec3) Sub(b Vec3) Vec3 { return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z} }

// 腿的编号：左前、右前、左后、右后。
const (
	FL = iota // front-left
	FR        // front-right
	RL        // rear-left
	RR        // rear-right
)

// LegNames 与腿编号一一对应。
var LegNames = [4]string{"FL", "FR", "RL", "RR"}

// Gait 用相位表描述一个周期性步态：
// 腿 i 的相位 φ_i(t) = frac(Freq·t + Offsets[i])，
// φ_i ∈ [0, Duty) 为支撑相，φ_i ∈ [Duty, 1) 为摆动相。
type Gait struct {
	Name    string     `json:"name"`
	Freq    float64    `json:"freq"`    // 步态周期频率 Hz
	Duty    float64    `json:"duty"`    // 支撑相占比 β ∈ (0,1)
	Offsets [4]float64 `json:"offsets"` // 各腿相位偏移 ∈ [0,1)
}

// WalkGait 返回横向顺序爬行步态（lateral-sequence walk），
// 抬腿顺序 RL → FL → RR → FR，任意时刻至少 3 足支撑（β=0.75）。
func WalkGait(freq float64) Gait {
	return Gait{
		Name: "walk", Freq: freq, Duty: 0.75,
		Offsets: [4]float64{FL: 0.5, FR: 0.0, RL: 0.75, RR: 0.25},
	}
}

// TrotGait 返回对角小跑步态：对角腿对 (FL,RR) 与 (FR,RL) 同步，
// 任意时刻最少仅 2 足支撑，属于动态步态（静态判据下不稳定属预期）。
func TrotGait(freq float64) Gait {
	return Gait{
		Name: "trot", Freq: freq, Duty: 0.55,
		Offsets: [4]float64{FL: 0.0, FR: 0.5, RL: 0.5, RR: 0.0},
	}
}

// Phase 返回腿 leg 在时刻 t（相对于步态相位原点）的归一化相位 ∈ [0,1)。
func (g Gait) Phase(leg int, t float64) float64 {
	p := math.Mod(g.Freq*t+g.Offsets[leg], 1)
	if p < 0 {
		p += 1
	}
	return p
}

// InStance 报告腿 leg 在时刻 t 是否处于支撑相。
func (g Gait) InStance(leg int, t float64) bool {
	return g.Phase(leg, t) < g.Duty
}

// Cycloid 返回摆线（cycloid）摆动轨迹上归一化进度 s∈[0,1] 处的足端位置。
//
// 水平分量：p(s) = p0 + (p1-p0)·(s − sin(2πs)/2π)
// 竖直分量：z(s) = z0 + (z1−z0)·(s − sin(2πs)/2π) + h·(1−cos(2πs))/2
//
// 两个分量在 s=0 与 s=1 处的一阶导数均为零，因此抬腿与落地瞬间
// 足端速度（含垂直方向）连续收敛到 0，不存在硬性速度跳变。
func Cycloid(p0, p1 Vec3, h, s float64) Vec3 {
	sh := s - math.Sin(2*math.Pi*s)/(2*math.Pi)
	return Vec3{
		X: p0.X + (p1.X-p0.X)*sh,
		Y: p0.Y + (p1.Y-p0.Y)*sh,
		Z: p0.Z + (p1.Z-p0.Z)*sh + h*(1-math.Cos(2*math.Pi*s))/2,
	}
}

// CycloidVerticalVelocity 返回摆线轨迹竖直方向在进度 s 处的速度（m/s）。
// 解析式 dz/dt = h·π·sin(2πs)/swingDur，在 s=0 与 s=1 处恰为 0。
func CycloidVerticalVelocity(h, swingDur, s float64) float64 {
	return h * math.Pi * math.Sin(2*math.Pi*s) / swingDur
}
