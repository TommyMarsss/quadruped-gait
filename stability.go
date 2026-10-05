package main

import (
	"fmt"
	"math"
	"sort"
)

// ConvexHull 用 Andrew 单调链算法求二维点集凸包，
// 返回逆时针排列的顶点（不重复首尾点）。点数 < 3 时原样返回。
func ConvexHull(pts []Vec2) []Vec2 {
	if len(pts) < 3 {
		return append([]Vec2(nil), pts...)
	}
	p := append([]Vec2(nil), pts...)
	sort.Slice(p, func(i, j int) bool {
		if p[i].X != p[j].X {
			return p[i].X < p[j].X
		}
		return p[i].Y < p[j].Y
	})
	cross := func(o, a, b Vec2) float64 {
		return (a.X-o.X)*(b.Y-o.Y) - (a.Y-o.Y)*(b.X-o.X)
	}
	lower := make([]Vec2, 0, len(p))
	for _, pt := range p {
		for len(lower) >= 2 && cross(lower[len(lower)-2], lower[len(lower)-1], pt) <= 0 {
			lower = lower[:len(lower)-1]
		}
		lower = append(lower, pt)
	}
	upper := make([]Vec2, 0, len(p))
	for i := len(p) - 1; i >= 0; i-- {
		for len(upper) >= 2 && cross(upper[len(upper)-2], upper[len(upper)-1], p[i]) <= 0 {
			upper = upper[:len(upper)-1]
		}
		upper = append(upper, p[i])
	}
	return append(lower[:len(lower)-1], upper[:len(upper)-1]...)
}

// StabilityMargin 计算重心投影 p 相对支撑多边形 hull（逆时针）的
// 静态稳定裕度：p 到各边的最小有符号距离，多边形内为正、外为负。
// hull 顶点数 < 3 时返回 math.Inf(-1)（支撑点不足以构成多边形）。
func StabilityMargin(p Vec2, hull []Vec2) float64 {
	if len(hull) < 3 {
		return math.Inf(-1)
	}
	margin := math.Inf(1)
	for i := range hull {
		a, b := hull[i], hull[(i+1)%len(hull)]
		ex, ey := b.X-a.X, b.Y-a.Y
		l := math.Hypot(ex, ey)
		if l == 0 {
			continue
		}
		// 逆时针多边形内部在每条边的左侧，有符号距离 = cross(e, p-a)/|e|
		d := (ex*(p.Y-a.Y) - ey*(p.X-a.X)) / l
		if d < margin {
			margin = d
		}
	}
	return margin
}

// SupportStability 给定支撑足位置集合与重心投影，返回稳定裕度与是否
// 满足静态稳定判据：至少 3 个支撑足构成多边形，且重心投影落在多边形内
// 且到各边距离不小于 minMargin。
func SupportStability(stanceFeet []Vec2, com Vec2, minMargin float64) (margin float64, stable bool) {
	if len(stanceFeet) < 3 {
		return math.Inf(-1), false
	}
	margin = StabilityMargin(com, ConvexHull(stanceFeet))
	return margin, margin >= minMargin
}

// GaitAnalysis 是对一组步态参数的可行性预检报告。
type GaitAnalysis struct {
	Gait             Gait     `json:"gait"`
	Speed            float64  `json:"speed"`
	Stride           float64  `json:"stride"`        // 步幅 = speed / freq
	MinStanceLegs    int      `json:"minStanceLegs"` // 一个周期内最少同时支撑足数
	StaticallyStable bool     `json:"staticallyStable"`
	Issues           []string `json:"issues"`
}

// AnalyzeGait 在不运行仿真的情况下检查步态参数是否会产生几何上必然
// 倾覆的构型：同时支撑足数不足、步幅超出腿部工作空间、非法参数等。
// maxStride 为单腿工作空间允许的最大步幅。
func AnalyzeGait(g Gait, speed, maxStride float64) GaitAnalysis {
	a := GaitAnalysis{Gait: g, Speed: speed, MinStanceLegs: 4, StaticallyStable: true}
	if g.Freq <= 0 {
		a.Issues = append(a.Issues, fmt.Sprintf("非法步频 %.2f Hz（必须为正）", g.Freq))
	}
	if g.Duty <= 0 || g.Duty >= 1 {
		a.Issues = append(a.Issues, fmt.Sprintf("非法支撑相占比 β=%.2f（必须在 (0,1) 内）", g.Duty))
	}
	if len(a.Issues) > 0 {
		a.StaticallyStable = false
		a.MinStanceLegs = 0
		return a
	}

	// 采样一个完整周期，统计任意相位下处于支撑相的最少腿数。
	const samples = 4096
	for k := 0; k < samples; k++ {
		phi := float64(k) / samples
		stance := 0
		for leg := 0; leg < 4; leg++ {
			if math.Mod(phi+g.Offsets[leg], 1) < g.Duty {
				stance++
			}
		}
		if stance < a.MinStanceLegs {
			a.MinStanceLegs = stance
		}
	}
	if a.MinStanceLegs < 3 {
		a.StaticallyStable = false
		a.Issues = append(a.Issues, fmt.Sprintf(
			"同时支撑足数最少为 %d（<3）：支撑相占比 β=%.2f 过低/摆动相占比过大，静态稳定性判据不满足（仅可作为动态步态使用）",
			a.MinStanceLegs, g.Duty))
	}

	a.Stride = speed / g.Freq
	if a.Stride > maxStride {
		a.StaticallyStable = false
		a.Issues = append(a.Issues, fmt.Sprintf(
			"步幅 %.3f m 超出腿部工作空间上限 %.3f m（速度 %.2f m/s 相对步频 %.2f Hz 过高，等价于步频过低或速度过高）",
			a.Stride, maxStride, speed, g.Freq))
	}
	return a
}
