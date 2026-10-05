package gait

import (
	"math"
	"testing"
)

// TestSwitchWaitsForCurrentPhase 是核心约束测试：
// 在某只脚摆动相进行中请求切换时，相位表绝不能立刻替换，
// 必须等到在摆脚全部完成当前摆动、四足着地后才切换；
// 且切换瞬间不允许任何足端位置跳变。
func TestSwitchWaitsForCurrentPhase(t *testing.T) {
	geo := DefaultGeometry()
	wp := Params{Type: Walk, Frequency: 0.8, DutyFactor: WalkDuty}
	sim := NewSimulator(geo, wp, 0.18)

	// 在 walk 的第一个摆动窗口（FL 摆动，[0,0.3125)）中点请求切换。
	const reqT = 0.15
	tp := Params{Type: Trot, Frequency: 2.0, DutyFactor: TrotDuty}
	sim.RequestSwitch(reqT, tp, 0.45)

	// 逐帧推进到切换完成；记录每只脚的连续位置轨迹。
	const dt = 0.002
	var switchT float64 = math.NaN()
	var prevPos [4]Vec3
	havePrev := false
	maxJump := 0.0
	sawPrematureGait := false

	for tk := dt; tk <= 4.0; tk += dt {
		f := sim.Sample(tk)
		pos := sim.footPositions(tk)

		// 先同步切换时刻，再判断本帧步态，避免同帧内状态已切换
		// 而 switchT 尚未更新造成误报。
		if math.IsNaN(switchT) {
			for _, sw := range sim.Switches() {
				switchT = sw.SwitchT
			}
		}

		// 请求之后、真正切换之前，步态名必须仍是 walk，
		// 且 FL 不能在摆动途中被"传送"或中断。
		if tk >= reqT && math.IsNaN(switchT) {
			if f.Gait != "walk" {
				sawPrematureGait = true
			}
		}

		// 检测相邻采样间所有脚的位置增量（摆动脚允许正常轨迹位移，
		// 但任何脚都不允许超过"单步物理上限"的瞬间跳变）。
		if havePrev {
			for i := range pos {
				dx := pos[i].X - prevPos[i].X
				dy := pos[i].Y - prevPos[i].Y
				dz := pos[i].Z - prevPos[i].Z
				d := math.Sqrt(dx*dx + dy*dy + dz*dz)
				// 0.002s 内足端速度物理上限取 3 m/s → 6mm；传送会远超。
				if d > 3*dt+1e-9 {
					t.Errorf("t=%.3f 腿 %d 位置增量 %.4fm 异常（疑似传送）", tk, i, d)
				}
				if d > maxJump {
					maxJump = d
				}
			}
		}
		prevPos = pos
		havePrev = true

		if math.IsNaN(switchT) {
			for _, sw := range sim.Switches() {
				switchT = sw.SwitchT
			}
		}
		if !math.IsNaN(switchT) && tk > switchT+0.05 {
			break
		}
	}

	if sawPrematureGait {
		t.Fatal("切换请求后在当前相完成前就替换了相位表")
	}
	if math.IsNaN(switchT) {
		t.Fatal("切换始终未发生")
	}
	// 切换不得早于：请求时刻 + FL 完成其当前摆动所需时间。
	// FL 在 0 起步、0.3125 落脚，故切换必须 >= 0.3125（严格晚于请求 0.15）。
	if switchT < 0.3125-1e-9 {
		t.Errorf("切换时刻 %.4f 早于在摆脚 FL 的落脚时刻 0.3125", switchT)
	}
	if switchT <= reqT {
		t.Errorf("切换在请求时刻立即生效（%.4f <= %.4f），未等待", switchT, reqT)
	}

	// 切换事件本身的属性：四足着地、零位置跳变。
	sw := sim.Switches()[0]
	if !sw.AllGrounded {
		t.Error("执行切换时并非四足全部着地")
	}
	if sw.MaxJump > 1e-9 {
		t.Errorf("切换瞬间足端位置跳变 %.3e", sw.MaxJump)
	}
	if sw.To != "trot" || sw.From != "walk" {
		t.Errorf("切换方向错误：%s -> %s", sw.From, sw.To)
	}
}

// TestSwitchNotDuringAnySwing 在多个请求时刻验证：
// 切换只在旧相位表下四足全部着地时执行；切换前任一在摆腿都
// 完整地走到了落脚（没有在摆动途中被打断/传送）。
//
// 注意：切换到 trot 后，新相位表的第一批对角腿在 switchT 立即
// 起摆是正确行为（从锚点 u=0 连续抬起），不属于"切换中摆动"。
func TestSwitchNotDuringAnySwing(t *testing.T) {
	requestTimes := []float64{0.05, 0.28, 0.5, 0.9, 1.17, 1.8, 2.6, 3.99}
	for _, reqT := range requestTimes {
		geo := DefaultGeometry()
		wp := WalkParams(0.8)
		sim := NewSimulator(geo, wp, 0.15)
		tp := TrotParams(1.6)
		sim.RequestSwitch(reqT, tp, 0.3)

		var switchT float64 = math.NaN()
		// 记录切换前每条腿最后一次落脚时刻与最晚抬脚时刻。
		for tk := 0.002; tk <= 5.0; tk += 0.002 {
			sim.advanceTo(tk)
			if len(sim.Switches()) > 0 {
				switchT = sim.Switches()[0].SwitchT
				break
			}
		}
		if math.IsNaN(switchT) {
			t.Fatalf("reqT=%.2f 切换未发生", reqT)
		}
		if switchT < reqT-1e-9 {
			t.Errorf("reqT=%.2f 切换时刻 %.3f 早于请求", reqT, switchT)
		}
		sw := sim.Switches()[0]
		if !sw.AllGrounded {
			t.Errorf("reqT=%.2f 事件记录的切换时刻非四足着地", reqT)
		}
		// 切换前记录的每次落脚都必须晚于/等于对应摆动起点，
		// 即切换前不存在"起了摆却没落脚"的腿：用落脚事件数与
		// walk 周期内的摆动调度一致性间接验证——更直接的是
		// 检查 switchT 前一瞬所有腿已不在旧表摆动中。
		// 重新构造一台跑到 switchT-ε 的仿真器查看旧状态。
		sim2 := NewSimulator(geo, wp, 0.15)
		sim2.RequestSwitch(reqT, tp, 0.3)
		sim2.advanceTo(switchT - 1e-6)
		for i := LegID(0); i < 4; i++ {
			if sim2.legs[i].inSwing {
				t.Errorf("reqT=%.2f 切换前一瞬 %.4f 腿 %d 仍在旧表摆动中",
					reqT, switchT, i)
			}
		}
	}
}

// TestSwingContinuesUnaffectedBeforeRequest 验证请求登记本身
// （未到点）不干扰既定时序：不登记切换时 FL 落脚时刻不变。
func TestSwingContinuesUnaffectedBeforeRequest(t *testing.T) {
	geo := DefaultGeometry()
	wp := Params{Type: Walk, Frequency: 0.8, DutyFactor: WalkDuty}

	simA := NewSimulator(geo, wp, 0.15)
	simB := NewSimulator(geo, wp, 0.15)
	// A 在 6s 才请求切换（提前登记）；B 不请求。
	simA.RequestSwitch(6.0, Params{Type: Trot, Frequency: 2, DutyFactor: TrotDuty}, 0.4)

	const dt = 0.001
	var firstTdA, firstTdB float64 = math.NaN(), math.NaN()
	for tk := dt; tk < 1.0; tk += dt {
		simA.advanceTo(tk)
		simB.advanceTo(tk)
		if math.IsNaN(firstTdA) && len(simA.Touchdowns()) > 0 {
			firstTdA = simA.Touchdowns()[0].T
		}
		if math.IsNaN(firstTdB) && len(simB.Touchdowns()) > 0 {
			firstTdB = simB.Touchdowns()[0].T
		}
	}
	if math.Abs(firstTdA-firstTdB) > 1e-9 {
		t.Errorf("未到点的切换请求干扰了既定时序：%.6f vs %.6f", firstTdA, firstTdB)
	}
	if math.Abs(firstTdB-0.3125) > 1e-6 {
		t.Errorf("FL 首次落脚时刻 %.4f，期望 0.3125", firstTdB)
	}
}
