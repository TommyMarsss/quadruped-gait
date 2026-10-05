package gait

import (
	"math"
	"testing"
)

// 已知支撑构型（单位米，机身典型尺寸 a=0.25, b=0.18）。
func fourCornerFeet() []XY {
	return []XY{
		{0.25, 0.18},   // FL
		{0.25, -0.18},  // FR
		{-0.25, 0.18},  // BL
		{-0.25, -0.18}, // BR
	}
}

func TestStaticStabilityStableConfig(t *testing.T) {
	// 重心位于四足矩形中心：到最近边（y=±0.18）距离 0.18m，稳定。
	r := StaticStability(XY{0, 0}, fourCornerFeet())
	if !r.Stable {
		t.Fatalf("中心构型应稳定，却报告：%s", r.Reason)
	}
	if math.Abs(r.Margin-0.18) > 1e-9 {
		t.Errorf("裕度=%.4f，期望 0.18", r.Margin)
	}
	if r.SupportCount != 4 || len(r.Polygon) != 4 {
		t.Errorf("支撑数=%d 凸包顶点=%d，期望 4/4", r.SupportCount, len(r.Polygon))
	}
}

func TestStaticStabilityTriangleConfigs(t *testing.T) {
	// 去掉左前足，支撑三角形为 FR/BL/BR。形心 (0.25-0.25-0.25)/3=-0.0833,
	// y=( -0.18+0.18-0.18)/3=-0.06；把重心放在三角形内部深处。
	tri := []XY{{0.25, -0.18}, {-0.25, 0.18}, {-0.25, -0.18}}
	rIn := StaticStability(XY{-0.15, -0.05}, tri)
	if !rIn.Stable {
		t.Errorf("三角形内点应稳定，却报告：%s (裕度 %v)", rIn.Reason, rIn.Margin)
	}
	// 重心明显在三角形外侧（右前方，越过 FR-BL 斜边）。
	rOut := StaticStability(XY{0.3, 0.25}, tri)
	if rOut.Stable || rOut.Margin >= 0 {
		t.Errorf("三角形外点应不稳定且裕度为负，得到 stable=%v margin=%v",
			rOut.Stable, rOut.Margin)
	}
}

func TestStaticStabilityBoundaryAndDegenerate(t *testing.T) {
	feet := fourCornerFeet()
	// 重心恰在支撑边上（y=0.18）：裕度为 0，必须判为不稳定（不能含混为稳定）。
	rEdge := StaticStability(XY{0.0, 0.18}, feet)
	if rEdge.Stable || rEdge.Margin > 0 {
		t.Errorf("重心在边上应判不稳定/零裕度，得到 stable=%v margin=%v",
			rEdge.Stable, rEdge.Margin)
	}
	// 重心在多边形外。
	rOut := StaticStability(XY{0, 0.6}, feet)
	if rOut.Stable || rOut.Margin >= 0 {
		t.Errorf("多边形外应不稳定且裕度为负，得到 %v", rOut.Margin)
	}
	// 裕度低于阈值但仍在内部：取距边 1cm 的点（< MarginThreshold=2cm）。
	rThin := StaticStability(XY{0, 0.17}, feet)
	if rThin.Stable {
		t.Error("裕度 0.01m 小于阈值，应报告裕度不足")
	}
	// 支撑足不足三只。
	for _, n := range []int{0, 1, 2} {
		r := StaticStability(XY{0, 0}, feet[:n])
		if r.Stable {
			t.Errorf("%d 只支撑足不应稳定", n)
		}
	}
	// 三只共线脚：凸包退化为线段。
	rCol := StaticStability(XY{0, 0}, []XY{{-0.3, 0}, {0, 0}, {0.3, 0}})
	if rCol.Stable {
		t.Error("共线支撑足不应判稳")
	}
}

// TestFeasibilityBoundaryCombos 验证步频/占空比边界组合被正确分类。
func TestFeasibilityBoundaryCombos(t *testing.T) {
	cases := []struct {
		name     string
		p        Params
		static   bool
		feasible bool
		minSup   int
	}{
		{"walk β=0.75 f=1.0：极限三足支撑，静态可行",
			Params{Walk, 1.0, 0.75}, true, true, 3},
		{"walk β=0.80 f=1.2：存在四足重叠窗口，静态可行",
			Params{Walk, 1.2, 0.80}, true, true, 3},
		{"walk β=0.70：摆动重叠，仅两足支撑，静态不可行",
			Params{Walk, 1.0, 0.70}, true, false, 2},
		{"walk β=0.60：摆动占比过大，静态不可行",
			Params{Walk, 0.8, 0.60}, true, false, 2},
		{"walk β=0.75 但步频 3.0 超准静态上限，不可行",
			Params{Walk, 3.0, 0.75}, true, false, 3},
		{"walk β=0.75 步频恰在上限 1.5，可行",
			Params{Walk, 1.5, 0.75}, true, true, 3},
		{"trot β=0.5：静态标准下不可行（两足支撑）",
			Params{Trot, 2.0, 0.50}, true, false, 2},
		{"trot β=0.5：作为动态步态放行（仅查物理合法性）",
			Params{Trot, 2.0, 0.50}, false, true, 2},
		{"trot β=0.55：对角摆动仍重叠，最少两足",
			Params{Trot, 2.0, 0.55}, true, false, 2},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			r := CheckFeasibility(c.p, c.static)
			if r.Feasible != c.feasible {
				t.Errorf("Feasible=%v 期望 %v；原因：%s",
					r.Feasible, c.feasible, r.Reason)
			}
			if r.MinSupport != c.minSup {
				t.Errorf("MinSupport=%d 期望 %d", r.MinSupport, c.minSup)
			}
			if !c.feasible && c.static && r.Reason == "" {
				t.Error("不可行组合必须给出明确原因")
			}
		})
	}
}

func TestFeasibilityInvalidParams(t *testing.T) {
	bad := []Params{
		{Walk, 0, 0.75},
		{Walk, -1, 0.75},
		{Walk, 1.0, 0},
		{Walk, 1.0, 1.2},
	}
	for _, p := range bad {
		if r := CheckFeasibility(p, true); r.Feasible {
			t.Errorf("非法参数 %+v 不应可行", p)
		}
	}
}

// TestDutyFactorContinuityBoundary 验证占空比在 0.75 附近的连续性：
// 略低于 0.75 立即出现两足支撑瞬间（几何上必然倾覆），
// 这是必须被检测到的边界，而不是静默生成倾覆步态。
func TestDutyFactorContinuityBoundary(t *testing.T) {
	justBelow := CheckFeasibility(Params{Walk, 1.0, 0.75 - 1e-3}, true)
	if justBelow.Feasible || justBelow.MinSupport >= 3 {
		t.Errorf("β=0.749 必须判为最少两足支撑/静态不可行，得到 %+v", justBelow)
	}
	atLimit := CheckFeasibility(Params{Walk, 1.0, 0.75}, true)
	if !atLimit.Feasible || atLimit.MinSupport != 3 {
		t.Errorf("β=0.75 边界应为三足支撑可行，得到 %+v", atLimit)
	}
}
