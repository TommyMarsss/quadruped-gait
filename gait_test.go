package main

import (
	"math"
	"testing"
)

// ---------- 摆线轨迹 ----------

func TestCycloidEndpointsAndPeak(t *testing.T) {
	p0 := Vec3{X: 0.1, Y: -0.2}
	p1 := Vec3{X: 0.4, Y: -0.2}
	const h = 0.05

	if got := Cycloid(p0, p1, h, 0); got != p0 {
		t.Errorf("s=0 应等于起点, 得到 %+v", got)
	}
	if got := Cycloid(p0, p1, h, 1); got != p1 {
		t.Errorf("s=1 应等于终点, 得到 %+v", got)
	}
	if z := Cycloid(p0, p1, h, 0.5).Z; math.Abs(z-h) > 1e-12 {
		t.Errorf("s=0.5 应达到峰值高度 %v, 得到 %v", h, z)
	}
}

func TestCycloidLandingVelocityConverges(t *testing.T) {
	const h, swingDur = 0.05, 0.25
	// 解析垂直速度在起止点必须严格为零（无硬性速度跳变）
	for _, s := range []float64{0, 1} {
		if v := CycloidVerticalVelocity(h, swingDur, s); math.Abs(v) > 1e-12 {
			t.Errorf("s=%v 处垂直速度应收敛到 0, 得到 %v", s, v)
		}
	}
	// 数值验证：落地前最后一个毫秒的垂直速度远小于峰值速度
	p0, p1 := Vec3{}, Vec3{X: 0.3}
	peakV := h * math.Pi / swingDur // s=0.5 处峰值
	vLate := math.Abs(CycloidVerticalVelocity(h, swingDur, 0.999))
	if vLate > 0.02*peakV {
		t.Errorf("落地前垂直速度 %v 未收敛（峰值 %v）", vLate, peakV)
	}
	// 水平速度同样在终点收敛到 0
	dEnd := Cycloid(p0, p1, h, 1).Sub(Cycloid(p0, p1, h, 0.999))
	vHoriz := math.Hypot(dEnd.X, dEnd.Y) / (0.001 * swingDur)
	peakHoriz := 2 * 0.3 / swingDur // dx/ds 峰值 = 2·stride
	if vHoriz > 0.02*peakHoriz {
		t.Errorf("落地前水平速度 %v 未收敛（峰值 %v）", vHoriz, peakHoriz)
	}
	// 轨迹必须光滑：相邻采样点间距有界（无瞬间传送）
	prev := Cycloid(p0, p1, h, 0)
	const N = 1000
	for k := 1; k <= N; k++ {
		cur := Cycloid(p0, p1, h, float64(k)/N)
		d := cur.Sub(prev)
		if step := math.Hypot(math.Hypot(d.X, d.Y), d.Z); step > 3.0*0.3/N {
			t.Fatalf("s=%v 附近轨迹不光滑，步长 %v", float64(k)/N, step)
		}
		prev = cur
	}
}

// ---------- 步态切换 ----------

// 请求切换时若某腿正处于摆动相，必须等其触地、四足均支撑后才切换，
// 且切换瞬间任何足端位置不得跳变。
func TestGaitSwitchWaitsForSwingCompletion(t *testing.T) {
	params := SimParams{HipX: 0.2, HipY: 0.12, BodyH: 0.32, StepH: 0.05, MinMargin: 0.02}
	sim := NewSim(params, WalkGait(1.0), func(float64) Vec2 { return Vec2{X: 0.2} })

	const dt = 0.005
	const reqT = 3.13 // walk@1Hz：t∈[3.00,3.25) 时 RL 正处于摆动相
	requested := false
	var prevFrame Frame
	switchedAt := -1.0

	for simT := 0.0; simT < 6.0; simT += dt {
		if !requested && simT >= reqT {
			sim.RequestSwitch(TrotGait(1.6))
			requested = true
		}
		f := sim.Step(dt)

		if requested && switchedAt < 0 {
			if sim.SwitchPending() && sim.Gait.Name != "walk" {
				t.Fatal("切换请求未生效前相位表不应改变")
			}
			if !sim.SwitchPending() { // 切换刚发生
				switchedAt = f.T
				// RL 摆动相在 t=3.25 结束，切换不得早于该时刻
				if f.T < 3.25-2*dt {
					t.Fatalf("切换发生于 t=%.3f，早于摆动腿触地时刻 3.25", f.T)
				}
				// 切换瞬间四足必须全部处于支撑相（脚跟着地）
				for i := 0; i < 4; i++ {
					if !f.Stance[i] {
						t.Fatalf("切换帧腿 %s 不在支撑相", LegNames[i])
					}
					if math.Abs(f.Feet[i].Z) > 1e-9 {
						t.Fatalf("切换帧腿 %s 足端离地 z=%v", LegNames[i], f.Feet[i].Z)
					}
					// 切换瞬间足端位置不得跳变
					d := f.Feet[i].Sub(prevFrame.Feet[i])
					if dist := math.Hypot(math.Hypot(d.X, d.Y), d.Z); dist > 0.01 {
						t.Fatalf("切换瞬间腿 %s 足端跳变 %v m", LegNames[i], dist)
					}
				}
			}
		}
		prevFrame = f
	}
	if switchedAt < 0 {
		t.Fatal("6 秒内切换未生效")
	}
	if len(sim.Switches) != 1 || sim.Switches[0].From != "walk" || sim.Switches[0].To != "trot" {
		t.Fatalf("切换记录异常: %+v", sim.Switches)
	}
	if math.Abs(sim.Switches[0].T-3.25) > 2*dt {
		t.Fatalf("切换时刻 %.4f 与预期 3.25 不符", sim.Switches[0].T)
	}
}

// 全程（含切换前后）支撑足在世界系中不得移动——支撑足滑移/传送即缺陷。
func TestStanceFeetNeverSlide(t *testing.T) {
	params := SimParams{HipX: 0.2, HipY: 0.12, BodyH: 0.32, StepH: 0.05, MinMargin: 0.02}
	sim := NewSim(params, WalkGait(1.0), func(t float64) Vec2 {
		if t < 4 {
			return Vec2{X: 0.2}
		}
		return Vec2{X: 0.35}
	})
	const dt = 0.005
	var prev Frame
	for simT := 0.0; simT < 10.0; simT += dt {
		if simT >= 4.0 && simT-dt < 4.0 {
			sim.RequestSwitch(TrotGait(1.6))
		}
		f := sim.Step(dt)
		if simT > 0 {
			for i := 0; i < 4; i++ {
				if f.Stance[i] && prev.Stance[i] {
					d := f.Feet[i].Sub(prev.Feet[i])
					if dist := math.Hypot(d.X, d.Y); dist > 1e-12 {
						t.Fatalf("t=%.3f 支撑腿 %s 滑移 %v m", f.T, LegNames[i], dist)
					}
				}
			}
		}
		prev = f
	}
}

// ---------- 静态稳定性判据 ----------

func TestStaticStabilityKnownConfigurations(t *testing.T) {
	// 已知支撑构型：±0.2 × ±0.15 矩形四足支撑
	square := []Vec2{{0.2, 0.15}, {0.2, -0.15}, {-0.2, 0.15}, {-0.2, -0.15}}

	m, ok := SupportStability(square, Vec2{0, 0}, 0.02)
	if !ok || math.Abs(m-0.15) > 1e-9 {
		t.Errorf("重心在矩形中心：应稳定且裕度=0.15, 得到 margin=%v stable=%v", m, ok)
	}
	m, ok = SupportStability(square, Vec2{0.10, 0.05}, 0.02)
	if !ok || math.Abs(m-0.10) > 1e-9 {
		t.Errorf("偏心重心：应稳定且裕度=0.10, 得到 margin=%v stable=%v", m, ok)
	}
	if _, ok = SupportStability(square, Vec2{0.25, 0}, 0.02); ok {
		t.Error("重心投影在支撑多边形外：应判为不稳定")
	}
	// 重心在多边形内但裕度低于阈值
	if _, ok = SupportStability(square, Vec2{0.19, 0}, 0.02); ok {
		t.Error("裕度 0.01 < 阈值 0.02：应判为不稳定")
	}
	// 仅两点支撑（如 trot 对角支撑）：不构成多边形，静态判据必须判不稳定
	if _, ok = SupportStability([]Vec2{{0.2, 0.15}, {-0.2, -0.15}}, Vec2{0, 0}, 0.02); ok {
		t.Error("双点支撑：静态判据应判为不稳定")
	}
	// 三角形支撑，重心在外
	tri := []Vec2{{0.2, 0.15}, {0.2, -0.15}, {-0.2, 0}}
	if _, ok = SupportStability(tri, Vec2{-0.15, 0.1}, 0.0); ok {
		t.Error("三角形支撑、重心投影在外：应判为不稳定")
	}
	if _, ok = SupportStability(tri, Vec2{0.05, 0}, 0.0); !ok {
		t.Error("三角形支撑、重心投影在内：应判为稳定")
	}
}

// ---------- 步态参数边界 ----------

func TestGaitParameterBoundaryDetection(t *testing.T) {
	const maxStride = 0.45

	// 合法 walk：β=0.75，任意时刻 ≥3 足支撑
	a := AnalyzeGait(WalkGait(1.0), 0.2, maxStride)
	if !a.StaticallyStable || a.MinStanceLegs != 3 || len(a.Issues) != 0 {
		t.Errorf("标准 walk 应判为静态稳定: %+v", a)
	}

	// 摆动相占比过大（β=0.6）：出现双足支撑区间 → 必须识别为不稳定
	bad := WalkGait(1.0)
	bad.Duty = 0.6
	a = AnalyzeGait(bad, 0.2, maxStride)
	if a.StaticallyStable || a.MinStanceLegs >= 3 || len(a.Issues) == 0 {
		t.Errorf("β=0.6 的 walk 应判为静态不稳定: %+v", a)
	}

	// β=0.45：支撑进一步崩塌
	bad.Duty = 0.45
	a = AnalyzeGait(bad, 0.2, maxStride)
	if a.StaticallyStable || a.MinStanceLegs >= 3 {
		t.Errorf("β=0.45 应判为静态不稳定且最少支撑足<3: %+v", a)
	}

	// trot：双足对角支撑，静态判据下必须明确报告（动态步态）
	a = AnalyzeGait(TrotGait(1.6), 0.35, maxStride)
	if a.StaticallyStable || a.MinStanceLegs != 2 || len(a.Issues) == 0 {
		t.Errorf("trot 应被报告为静态不稳定（动态步态）: %+v", a)
	}

	// 步频相对速度过低（等价于速度过高）→ 步幅超出工作空间
	a = AnalyzeGait(WalkGait(0.5), 0.4, maxStride)
	if a.StaticallyStable || len(a.Issues) == 0 {
		t.Errorf("步幅 0.8m 超出工作空间应被识别: %+v", a)
	}

	// 非法参数
	a = AnalyzeGait(Gait{Name: "bad", Freq: 0, Duty: 1.2}, 0.2, maxStride)
	if a.StaticallyStable || len(a.Issues) == 0 {
		t.Errorf("非法 freq/β 必须被报告: %+v", a)
	}
}

// ---------- 凸包 ----------

func TestConvexHull(t *testing.T) {
	pts := []Vec2{{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0.5, 0.5}, {0.5, 0.1}}
	h := ConvexHull(pts)
	if len(h) != 4 {
		t.Fatalf("单位正方形+内部点的凸包应有 4 个顶点, 得到 %d", len(h))
	}
	// 逆时针 ⇒ 内部点有符号距离为正
	if m := StabilityMargin(Vec2{0.5, 0.5}, h); m <= 0 {
		t.Errorf("中心点裕度应为正, 得到 %v", m)
	}
}
