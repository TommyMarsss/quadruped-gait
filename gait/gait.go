package gait

// GaitType 标识步态类型。
type GaitType int

const (
	Walk GaitType = iota // 爬行（静态稳定步行），默认 duty factor 0.75
	Trot                 // 对角小跑，默认 duty factor 0.5（对角足成对动作）
)

func (g GaitType) String() string {
	if g == Trot {
		return "trot"
	}
	return "walk"
}

// Params 是一组步态参数。
type Params struct {
	Type       GaitType
	Frequency  float64 // 步频 f：每秒完成的步态周期数（Hz）
	DutyFactor float64 // 占空比 β：每周期内单足处于支撑相的比例
}

// Duration 返回单个步态周期的时长 T = 1/f（秒）。
func (p Params) Duration() float64 { return 1 / p.Frequency }

// String 返回步态名称。
func (p Params) String() string { return p.Type.String() }

// 默认/建议步态参数。
const (
	// WalkDuty 是四拍爬行的理论最小占空比：任意时刻恰好三足支撑。
	// 但此时四足支撑窗口长度 (4β-3)T=0，质心只能在换脚瞬间贴边
	// 穿越共用边（McGhee 静态极限，裕度为零）。
	WalkDuty = 0.75
	// StableWalkDuty 留出非零四足重叠窗口 (β-0.75)T=0.10T，
	// 质心可在该窗口内连续转移到下一三足三角形，工程准静态爬行采用它。
	StableWalkDuty = 0.85
	TrotDuty       = 0.50 // 小跑：摆动支撑各半，对角足同时起落
)

// DefaultParams 返回给定步态的典型参数。
func DefaultParams(t GaitType) Params {
	if t == Trot {
		return Params{Type: Trot, Frequency: 2.0, DutyFactor: TrotDuty}
	}
	return Params{Type: Walk, Frequency: 1.0, DutyFactor: WalkDuty}
}

// nominalOffsets 返回各腿摆动相开始时刻在周期内的标称归一化偏移 phi_i
// （相位在偏移点进入摆动相）。
//
// Walk（爬行情报，四拍序 FL -> BR -> FR -> BL，见 McGhee 1968）：
//
//	phi_FL=0, phi_BR=1/4, phi_FR=1/2, phi_BL=3/4
//
// 配合 β=0.75 时任意时刻恰有 3 只脚支撑，重心始终位于支撑三角形内。
//
// Trot（对角小跑）：对角足 {FL,BR} 与 {FR,BL} 成对、相位差半周期：
//
//	phi_FL=phi_BR=0, phi_FR=phi_BL=1/2
func nominalOffsets(t GaitType) [4]float64 {
	if t == Trot {
		return [4]float64{0, 0.5, 0.5, 0}
	}
	return [4]float64{0, 0.5, 0.75, 0.25}
}

// IsSwing 在给定周期相位 phase ∈ [0,1) 下判断第 leg 条腿是否处于摆动相。
// 摆动相区间为 [phi_i, phi_i + 1-β)（模 1），其余为支撑相。
func (p Params) IsSwing(leg LegID, phase float64) bool {
	s := wrap01(phase - p.offset(leg))
	return s < 1-p.DutyFactor
}

// offset 返回某条腿的摆动相起始偏移。
func (p Params) offset(leg LegID) float64 { return nominalOffsets(p.Type)[leg] }

// nextPhaseEvent 返回从相位 phase 出发，某条腿下一次"相切换事件"
// （摆动->支撑 即落脚，或 支撑->摆动 即抬脚）发生时的相位（可能 >1）
// 以及该事件是否为落脚事件。
func (p Params) nextPhaseEvent(leg LegID, phase float64) (at float64, touchdown bool) {
	o := p.offset(leg)
	swingEnd := o + (1 - p.DutyFactor) // 落脚相位
	// 把两个事件点折到当前周期/下一周期，取严格大于 phase 的最近者。
	lift := o
	for lift <= phase {
		lift += 1
	}
	land := swingEnd
	for land <= phase {
		land += 1
	}
	if lift < land {
		return lift, false
	}
	return land, true
}

// feasibility 与稳定性裕度相关的参数校验在 stability.go 中实现。
