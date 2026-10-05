package gait

import "math"

// 本文件实现足端摆动相轨迹。
//
// 水平方向采用"修正摆线（cycloid）"起步/收步规律，垂直方向采用
// 摆线拱。摆线规律的关键性质是：s(0)=s'(0)=s''(0)=0 且
// s(1)=1、s'(1)=s''(1)=0，因此足端在抬脚与落脚瞬间的位置、
// 速度、加速度都与支撑相连续衔接，落脚时不存在垂直速度的硬性跳变。

// SwingProfile 返回摆动相内 u∈[0,1] 处的归一化摆线进度 s(u)。
//
//	s(u) = u - sin(2πu)/(2π)
//
// 它满足 s(0)=0, s(1)=1，且一阶、二阶导数在两端均为 0：
//
//	s'(u) = 1 - cos(2πu)
//	s''(u) = 2π·sin(2πu)
func SwingProfile(u float64) float64 {
	if u <= 0 {
		return 0
	}
	if u >= 1 {
		return 1
	}
	return u - math.Sin(2*math.Pi*u)/(2*math.Pi)
}

// SwingProfileDeriv 返回 ds/du。
func SwingProfileDeriv(u float64) float64 {
	if u < 0 || u > 1 {
		return 0
	}
	return 1 - math.Cos(2*math.Pi*u)
}

// LiftHeight 返回摆动相内 u∈[0,1] 处的归一化离地高度 h(u)∈[0,1]。
//
// 采用摆线拱（周期减半的正弦半波积分形式）：
//
//	h(u) = (1 - cos(2πu))/2
//
// 有 h(0)=h(1)=0 且 h'(0)=h'(1)=0，故落脚垂直速度为零。
// 在 u=0.5 处达到峰值 1。
func LiftHeight(u float64) float64 {
	if u <= 0 || u >= 1 {
		return 0
	}
	return (1 - math.Cos(2*math.Pi*u)) / 2
}

// SwingFoot 计算单只脚在一次摆动相中的足端目标位置与速度。
//
// 入参：
//   - u: 摆动相进度 [0,1]（0=刚抬脚，1=刚落脚）
//   - p0: 抬脚点（世界系地面位置，Z=0）
//   - p1: 预定落地点（世界系地面位置，Z=0）
//   - liftHeight: 摆动最大抬脚高度 H（米）
//   - swingTime: 摆动相持续时间 t_s = (1-β)T（秒）
//
// 返回足端位置（Z 为离地高度）与速度。速度由链式法则
// d/dt = (1/t_s)·d/du 得到，供连续性/落脚速度检测使用。
func SwingFoot(u float64, p0, p1 Vec3, liftHeight, swingTime float64) (pos, vel Vec3) {
	s := SwingProfile(u)
	ds := SwingProfileDeriv(u) / swingTime
	h := LiftHeight(u)
	// h'(u) = π·sin(2πu)
	dh := math.Pi * math.Sin(2*math.Pi*u) / swingTime

	pos = Vec3{
		X: p0.X + (p1.X-p0.X)*s,
		Y: p0.Y + (p1.Y-p0.Y)*s,
		Z: liftHeight * h,
	}
	vel = Vec3{
		X: (p1.X - p0.X) * ds,
		Y: (p1.Y - p0.Y) * ds,
		Z: liftHeight * dh,
	}
	return pos, vel
}
