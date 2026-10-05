package gait

import "math"

// ScenarioConfig 描述一段演示序列：静止爬行起步、爬行加速、
// 在切换点请求切换到对角小跑并继续运行。
type ScenarioConfig struct {
	WalkV0   float64 // 爬行初速度（m/s），0=从静止起步
	WalkV1   float64 // 爬行加速后的速度
	TrotV    float64 // 小跑目标速度
	WalkFreq float64 // 爬行步频 Hz
	TrotFreq float64 // 小跑步频 Hz
	SwitchAt float64 // 请求步态切换的时刻 s
	EndAfter float64 // 切换完成后继续运行时长 s
	DT       float64 // 采样间隔 s
}

// DefaultScenario 返回默认演示参数。
func DefaultScenario() ScenarioConfig {
	return ScenarioConfig{
		WalkV0: 0.0, WalkV1: 0.15, TrotV: 0.45,
		WalkFreq: 0.8, TrotFreq: 2.0,
		SwitchAt: 6.0, EndAfter: 5.0, DT: 0.02,
	}
}

// WalkParams / TrotParams 返回场景使用的步态参数。
// 爬行采用 StableWalkDuty=0.85 以获得非零四足支撑窗口。
func WalkParams(freq float64) Params {
	return Params{Type: Walk, Frequency: freq, DutyFactor: StableWalkDuty}
}

// TrotParams 返回对角小跑参数（动态步态，β=0.5）。
func TrotParams(freq float64) Params {
	return Params{Type: Trot, Frequency: freq, DutyFactor: TrotDuty}
}

// ScenarioResult 汇总一次场景仿真。
type ScenarioResult struct {
	Frames     []Frame
	Touchdowns []TouchdownEvent
	Switches   []SwitchEvent
	Geo        Geometry
	LiftHeight float64
	Duration   float64
}

// RunScenario 执行配置的完整序列。
//
// 时序：
//  1. 0~accelT：爬行从 v0 线性加速到 v1（斜坡长一个爬行周期）；
//  2. SwitchAt：登记 walk->trot 切换请求。生成器不会立即换表，
//     而是等待在摆腿全部完成当前摆动、四足着地并驻留后才切换；
//  3. 切换后一个小跑周期内把速度平滑加到 TrotV，继续运行 EndAfter。
func RunScenario(geo Geometry, cfg ScenarioConfig) (*ScenarioResult, error) {
	wp := WalkParams(cfg.WalkFreq)
	if f := CheckFeasibility(wp, true); !f.Feasible {
		return nil, errText(f.Reason)
	}
	tp := TrotParams(cfg.TrotFreq)
	// 小跑是动态步态：按非静态标准检查，仅校验物理合法性。
	if f := CheckFeasibility(tp, false); !f.Feasible {
		return nil, errText(f.Reason)
	}

	sim := NewSimulator(geo, wp, cfg.WalkV0)
	if cfg.WalkV1 != cfg.WalkV0 {
		sim.RampVelocity(0, wp.Duration(), cfg.WalkV1)
	}
	sim.RequestSwitch(cfg.SwitchAt, tp, cfg.TrotV)

	// 切换时刻事先未知（取决于"等待当前相完成"），轮询发现切换后定终点。
	tEnd := cfg.SwitchAt + cfg.EndAfter + 4 // 预留过渡上界
	frames := sim.Run(tEnd, cfg.DT)
	var swT float64 = math.NaN()
	for _, sw := range sim.Switches() {
		swT = sw.SwitchT
	}
	endT := swT + cfg.EndAfter
	if math.IsNaN(swT) {
		endT = tEnd
	}
	// 截断到 endT。
	cut := frames
	for k, f := range frames {
		if f.T > endT {
			cut = frames[:k]
			break
		}
	}
	return &ScenarioResult{
		Frames: cut, Touchdowns: sim.Touchdowns(), Switches: sim.Switches(),
		Geo: geo, LiftHeight: defaultLiftHeight, Duration: endT,
	}, nil
}

type simpleError string

func (e simpleError) Error() string { return string(e) }
func errText(s string) error        { return simpleError(s) }
