package gait

import (
	"fmt"
	"math"
	"sort"
)

// XY 是平面点。
type XY struct{ X, Y float64 }

// convexHull 用 Andrew 单调链算法求点集凸包（逆时针序）。
// 输入点数 <= 2 时原样返回（调用方需自行判稳）。
func convexHull(pts []XY) []XY {
	p := make([]XY, len(pts))
	copy(p, pts)
	sort.Slice(p, func(i, j int) bool {
		if p[i].X != p[j].X {
			return p[i].X < p[j].X
		}
		return p[i].Y < p[j].Y
	})
	if len(p) <= 2 {
		return p
	}
	cross := func(o, a, b XY) float64 {
		return (a.X-o.X)*(b.Y-o.Y) - (a.Y-o.Y)*(b.X-o.X)
	}
	low := make([]XY, 0, len(p))
	for _, q := range p {
		for len(low) >= 2 && cross(low[len(low)-2], low[len(low)-1], q) <= 0 {
			low = low[:len(low)-1]
		}
		low = append(low, q)
	}
	up := make([]XY, 0, len(p))
	for i := len(p) - 1; i >= 0; i-- {
		q := p[i]
		for len(up) >= 2 && cross(up[len(up)-2], up[len(up)-1], q) <= 0 {
			up = up[:len(up)-1]
		}
		up = append(up, q)
	}
	return append(low[:len(low)-1], up[:len(up)-1]...)
}

// StabilityResult 是单帧（或单组支撑构型）的静态稳定性结论。
type StabilityResult struct {
	Stable       bool    // 重心投影严格位于支撑多边形内且裕度不小于阈值
	Margin       float64 // 有符号稳定裕度（米）：内正外负；支撑足<3 或退化时为 NaN
	SupportCount int     // 支撑足数量
	Reason       string  // 不稳定原因（稳定时为空）
	Polygon      []XY    // 支撑多边形（凸包，逆时针）
}

// 静态稳定裕度阈值：重心投影距支撑多边形边界至少 2 cm 才算裕度充足。
const MarginThreshold = 0.02

// StaticStability 判断重心投影 com 是否落在支撑足构成的支撑多边形内。
//
// 判据（静态稳定裕度，McGhee & Frank, 1968）：在准静态运动中，
// 重心在水平面上的投影必须位于由所有着地足位置凸包构成的支撑多边形内；
// 其到多边形各边的最小有符号距离即静态稳定裕度 S。
// S>0 静态稳定，S≤0 倾覆；工程上另要求 S 不小于裕度阈值 MarginThreshold。
//
// 支撑足少于 3 只时不存在有界支撑多边形（线段/点无法约束平面倾覆），
// 直接判为静态不稳定（对角小跑这类动态步态即属此情形，
// 需依靠动量动态稳定，不在静态判据覆盖范围内）。
func StaticStability(com XY, supportFeet []XY) StabilityResult {
	res := StabilityResult{SupportCount: len(supportFeet), Margin: math.NaN()}
	if len(supportFeet) < 3 {
		res.Stable = false
		res.Reason = fmt.Sprintf("同时支撑的脚仅 %d 只（<3），不构成有界支撑多边形，静态稳定性不成立", len(supportFeet))
		return res
	}
	hull := convexHull(supportFeet)
	res.Polygon = hull
	if len(hull) < 3 {
		res.Reason = "支撑足共线，支撑多边形退化"
		return res
	}
	// 凸包按逆时针排列。点到每条支撑边所在直线的有符号距离为
	// cross(边向量, 点-边起点)/|边向量|：点在内侧（左侧）为正。
	minMargin := math.Inf(1)
	for i := 0; i < len(hull); i++ {
		a, b := hull[i], hull[(i+1)%len(hull)]
		cr := (b.X-a.X)*(com.Y-a.Y) - (b.Y-a.Y)*(com.X-a.X)
		edge := math.Hypot(b.X-a.X, b.Y-a.Y)
		if edge == 0 {
			continue
		}
		if sd := cr / edge; sd < minMargin {
			minMargin = sd
		}
	}
	res.Margin = minMargin
	switch {
	case minMargin <= 0:
		res.Stable = false
		res.Reason = fmt.Sprintf("重心投影位于支撑多边形之外（有符号裕度 %.3f m ≤ 0）", minMargin)
	case minMargin < MarginThreshold:
		res.Stable = false
		res.Reason = fmt.Sprintf("静态稳定裕度不足：%.3f m < 阈值 %.3f m", minMargin, MarginThreshold)
	default:
		res.Stable = true
	}
	return res
}

// GaitFeasibility 从步态参数层面检查"任意时刻最少同时支撑脚数"，
// 在生成轨迹之前拦截几何上必然倾覆的参数组合。
//
// 对周期性相位步态，把各腿摆动区间 [phi_i, phi_i+1-β) 铺在圆周上，
// 取足够密的相位采样，统计最少支撑脚数与最小静态裕度的上界可能性。
// 此处做保守的解析判断：
//   - 最少支撑脚数 n_min 直接由相位区间扫描得到（精确到事件点）；
//   - n_min < 3 时该参数组合无法保持静态稳定（如 trot 的 β=0.5，
//     或任意步态 β 过小、频率过高导致必须跑动态）。
//
// 注意：频率本身不改变相位几何，但步频越高留给姿态/质心调整的
// 时间越短，因此另设 MaxStaticFrequency 上限。
type FeasibilityResult struct {
	Feasible   bool
	MinSupport int // 一个周期内最少同时支撑脚数
	Reason     string
}

// MaxStaticFrequency 是准静态步态允许的最高步频（Hz）。
// 高于此值时摆动相时间过短、质心调整跟不上，静态假设失效。
const MaxStaticFrequency = 1.5

// CheckFeasibility 检查步态参数是否可用于静态稳定行走。
// requireStatic=true 时按静态标准（任意时刻 >=3 脚支撑且频率受限）判定；
// requireStatic=false 时（如明确要跑动态 trot）只检查参数物理合法性。
func CheckFeasibility(p Params, requireStatic bool) FeasibilityResult {
	switch {
	case p.Frequency <= 0:
		return FeasibilityResult{Reason: "步频必须为正"}
	case p.DutyFactor <= 0 || p.DutyFactor > 1:
		return FeasibilityResult{Reason: "占空比 β 必须落在 (0,1]"}
	}
	// 在圆周上精确扫描：每个相位区间内摆动脚数恒定，
	// 取所有事件点（抬脚/落脚）排序后的区间中点统计即可。
	offsets := nominalOffsets(p.Type)
	swingLen := 1 - p.DutyFactor
	bounds := make([]float64, 0, 8)
	for _, o := range offsets {
		bounds = append(bounds, wrap01(o), wrap01(o+swingLen))
	}
	sort.Float64s(bounds)
	nMin := 4
	for k := 0; k < len(bounds); k++ {
		x0 := bounds[k]
		x1 := bounds[(k+1)%len(bounds)]
		if k == len(bounds)-1 {
			x1 += 1 // 跨过 0 点的末段
		}
		mid := wrap01((x0 + x1) / 2)
		swing := 0
		for _, o := range offsets {
			if wrap01(mid-o) < swingLen {
				swing++
			}
		}
		if s := 4 - swing; s < nMin {
			nMin = s
		}
	}
	r := FeasibilityResult{MinSupport: nMin, Feasible: true}
	if !requireStatic {
		return r
	}
	if nMin < 3 {
		r.Feasible = false
		r.Reason = fmt.Sprintf("占空比 β=%.2f 时任意时刻最少仅 %d 只脚同时支撑（<3），"+
			"无法满足静态稳定性；请增大 β（walk 建议 ≥0.75）或改按动态步态处理", p.DutyFactor, nMin)
		return r
	}
	if p.Frequency > MaxStaticFrequency {
		r.Feasible = false
		r.Reason = fmt.Sprintf("步频 %.2f Hz 超过准静态上限 %.2f Hz，摆动相内无足够时间完成质心调整",
			p.Frequency, MaxStaticFrequency)
	}
	return r
}
