// Package gait 实现四足机器人的步态相位调度、摆线足端轨迹生成、
// 步态切换的连续性处理以及静态稳定性检测。
//
// 这是一个纯运动学（准静态）模型：只使用标准库，坐标单位为米，时间单位为秒。
package gait

import "math"

// Vec3 是世界坐标系（或机体系，视上下文而定）下的三维点。
// 约定：X 指向机身前进方向，Y 指向机身左侧，Z 竖直向上。
type Vec3 struct {
	X, Y, Z float64
}

// LegID 标识四条腿。
type LegID int

const (
	FL LegID = iota // 左前 front-left
	FR              // 右前 front-right
	BL              // 左后 back-left
	BR              // 右后 back-right
)

// LegNames 按 LegID 顺序给出腿的名字。
var LegNames = [4]string{"FL", "FR", "BL", "BR"}

func (l LegID) String() string { return LegNames[l] }

// Geometry 描述机身几何尺寸（米）。
type Geometry struct {
	HalfLength float64 // 髋关节前后距质心距离 a
	HalfWidth  float64 // 髋关节左右距中心线距离 b
	BodyHeight float64 // 可视化中机身距地面的高度（不参与稳定性计算）
}

// DefaultGeometry 返回一组小型四足平台的典型尺寸。
func DefaultGeometry() Geometry {
	return Geometry{HalfLength: 0.25, HalfWidth: 0.18, BodyHeight: 0.22}
}

// HipPosition 返回第 i 条腿髋关节在机体系中的位置。
func (g Geometry) HipPosition(i LegID) Vec3 {
	switch i {
	case FL:
		return Vec3{X: g.HalfLength, Y: g.HalfWidth}
	case FR:
		return Vec3{X: g.HalfLength, Y: -g.HalfWidth}
	case BL:
		return Vec3{X: -g.HalfLength, Y: g.HalfWidth}
	default:
		return Vec3{X: -g.HalfLength, Y: -g.HalfWidth}
	}
}

// wrap01 把任意相位折回到 [0,1)。
func wrap01(p float64) float64 {
	p = math.Mod(p, 1)
	if p < 0 {
		p += 1
	}
	return p
}
