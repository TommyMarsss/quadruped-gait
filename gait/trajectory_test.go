package gait

import (
	"math"
	"testing"
)

// touchdownTol 是落脚瞬间允许的速度残差上界（m/s）。
// 摆线规律解析上速度恰为 0；数值实现只允许浮点量级残差。
const touchdownTol = 1e-6

func TestCycloidProfileEndpoints(t *testing.T) {
	// 位置端点：s(0)=0, s(1)=1。
	if got := SwingProfile(0); math.Abs(got) > 1e-15 {
		t.Errorf("SwingProfile(0)=%v, 期望 0", got)
	}
	if got := SwingProfile(1); math.Abs(got-1) > 1e-15 {
		t.Errorf("SwingProfile(1)=%v, 期望 1", got)
	}
	// 一阶导在两端必须为 0（落脚/抬脚无速度跳变）。
	for _, u := range []float64{0, 1} {
		if got := SwingProfileDeriv(u); math.Abs(got) > 1e-15 {
			t.Errorf("SwingProfileDeriv(%v)=%v, 期望 0", u, got)
		}
	}
	// 抬脚高度 h 在两端为 0。
	for _, u := range []float64{0, 1} {
		if got := LiftHeight(u); math.Abs(got) > 1e-15 {
			t.Errorf("LiftHeight(%v)=%v, 期望 0", u, got)
		}
	}
	// h 在摆动中点达到峰值 1。
	if got := LiftHeight(0.5); math.Abs(got-1) > 1e-15 {
		t.Errorf("LiftHeight(0.5)=%v, 期望 1", got)
	}
}

func TestSwingFootTouchdownVelocityConverges(t *testing.T) {
	p0 := Vec3{X: 0.10, Y: 0.18}
	p1 := Vec3{X: 0.34, Y: 0.18}
	const liftH, swingT = 0.06, 0.3125

	// 起步瞬间：位于 p0，速度为 0。
	pos0, vel0 := SwingFoot(0, p0, p1, liftH, swingT)
	if d := math.Hypot(pos0.X-p0.X, pos0.Y-p0.Y); d > 1e-12 || pos0.Z != 0 {
		t.Errorf("摆动起点位置不连续：偏移 %.3e, z=%v", d, pos0.Z)
	}
	if sp := math.Sqrt(vel0.X*vel0.X + vel0.Y*vel0.Y + vel0.Z*vel0.Z); sp > 1e-12 {
		t.Errorf("摆动起点速度=%v，期望 0", sp)
	}

	// 落脚前一瞬 u→1⁻：水平/垂直速度都必须收敛到容差内，
	// 不能是硬性瞬间跳变（对比：线性/常速轨迹在落脚处速度非零）。
	prev := math.Inf(1)
	for k := 1; k <= 12; k++ {
		u := 1 - math.Pow10(-k)
		_, vel := SwingFoot(u, p0, p1, liftH, swingT)
		speed := math.Sqrt(vel.X*vel.X + vel.Y*vel.Y + vel.Z*vel.Z)
		if speed > prev {
			t.Errorf("速度在趋近落脚时非单调收敛：u=%.0e speed=%.3e > 前值 %.3e",
				1-u, speed, prev)
		}
		prev = speed
	}
	_, velTD := SwingFoot(1-1e-12, p0, p1, liftH, swingT)
	if math.Abs(velTD.Z) > touchdownTol {
		t.Errorf("落脚垂直速度 %.3e 超过容差 %.0e", velTD.Z, touchdownTol)
	}
	if math.Hypot(velTD.X, velTD.Y) > touchdownTol {
		t.Errorf("落脚水平速度 %.3e 超过容差 %.0e",
			math.Hypot(velTD.X, velTD.Y), touchdownTol)
	}

	// 落脚位置必须精确等于 p1 且 z=0，与随后的支撑相锚点一致。
	pos1, _ := SwingFoot(1, p0, p1, liftH, swingT)
	if d := math.Hypot(pos1.X-p1.X, pos1.Y-p1.Y); d > 1e-12 {
		t.Errorf("落脚位置偏移 %.3e（传送/跳变）", d)
	}
	if pos1.Z != 0 {
		t.Errorf("落脚高度 z=%v，期望 0", pos1.Z)
	}
}

func TestSwingFootVelocityContinuousAcrossSwing(t *testing.T) {
	p0 := Vec3{X: 0, Y: -0.18}
	p1 := Vec3{X: 0.2, Y: -0.18}
	const liftH, swingT = 0.06, 0.3
	// 全摆动相内速度必须有限、垂直速度先正后负且无突变。
	maxVz := 0.0
	const n = 10000
	for k := 0; k <= n; k++ {
		u := float64(k) / n
		_, vel := SwingFoot(u, p0, p1, liftH, swingT)
		if math.IsNaN(vel.X+vel.Y+vel.Z) || math.IsInf(vel.Z, 0) {
			t.Fatalf("u=%v 出现非有限速度", u)
		}
		if math.Abs(vel.Z) > maxVz {
			maxVz = math.Abs(vel.Z)
		}
	}
	// 峰值垂直速度量级合理（6cm 摆线拱、0.3s 摆时 ≈ π*H/T ≈ 0.63 m/s）。
	want := math.Pi * liftH / swingT
	if math.Abs(maxVz-want) > 0.02*want {
		t.Errorf("峰值垂直速度 %.3f 与摆线解析值 %.3f 偏差过大", maxVz, want)
	}
}

func TestScenarioTouchdownsAllConverge(t *testing.T) {
	res, err := RunScenario(DefaultGeometry(), DefaultScenario())
	if err != nil {
		t.Fatal(err)
	}
	if len(res.Touchdowns) == 0 {
		t.Fatal("未记录任何落脚事件")
	}
	for _, td := range res.Touchdowns {
		if math.Abs(td.Vz) > touchdownTol {
			t.Errorf("腿 %d 在 t=%.3f 落脚垂直速度 %.3e 超差", td.Leg, td.T, td.Vz)
		}
		if td.VXY > touchdownTol {
			t.Errorf("腿 %d 在 t=%.3f 落脚水平速度 %.3e 超差", td.Leg, td.T, td.VXY)
		}
		if td.Jump > 1e-9 {
			t.Errorf("腿 %d 在 t=%.3f 落脚位置跳变 %.3e", td.Leg, td.T, td.Jump)
		}
	}
}
