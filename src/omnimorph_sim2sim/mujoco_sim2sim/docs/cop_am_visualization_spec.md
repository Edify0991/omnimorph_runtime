# 可视化 CoP-AM 与支撑多边形边界（验证 mimic 成败是否源于 CoP 出界）

> **交接文档**：给本仓库（`jc01_deploy` / `omnimorph_sim2sim`）的实现 agent。
> **目标**：在 MuJoCo sim2sim 中可视化 **CoP-AM（含角动量项的 CoP 公式）** 与**支撑多边形边界**，使得跑 BeyondMimic mimic 几个动作时，能在仿真里直接看出"**成功/失败 ↔ CoP 是否出支撑多边形**"，并能**在训练前对重定向参考轨迹预测**哪些段会出界。
> **关键**：本仓库 `scripts/replay_mcap_com_support.py` **已实现**接触式 CoP 真值 + 支撑多边形 + pinocchio CoM + viewer 叠加。本任务是在它之上**新增 CoP-AM 预测式 + 出界判定**，不要重写已有部分。

---

## 0. 背景物理事实（一句话）

**CoP（压力中心）必须落在当前支撑多边形 $\mathcal P$ 内，否则足底翻转 → 倒下（失败）。** 因此"CoP 出界"就是"即将翻倒"的直接预测信号。我们要在仿真里把这个信号画出来。

---

## 1. 数学定义（照此实现，防误解）

### 1.1 符号

| 符号 | 含义 | 来源（本仓库） |
|---|---|---|
| $c\in\mathbb R^3$ | 质心 CoM | 已有 `PinocchioCom.compute(...)` |
| $\dot c,\ddot c$ | CoM 速度/加速度 | 有限差分（§1.4） |
| $z_c=c_z$ | CoM 高度（瞬时，**非常数**） | $c[2]$ |
| $\ddot c_z$ | CoM 垂直加速度（瞬时） | $\ddot c[2]$ |
| $k\in\mathbb R^3$ | **绕质心**的角动量 | pinocchio（§1.3） |
| $\dot k$ | 角动量率 | 有限差分 |
| $p_{\rm CoP}\in\mathbb R^2$ | 地面（$z{=}0$）上的压力中心水平坐标 | 本任务计算 |
| $m$ | 机器人总质量 | `model.body_mass` 求和（或 URDF） |
| $g=9.81$ | 重力 | — |
| $\hat z=(0,0,1)$ | 竖直单位向量 | — |

### 1.2 CoP-AM 恒等式（**核心公式，精确，变高适用**）

$$\boxed{\;p_{\rm CoP}^{xy}=c^{xy}-\frac{z_c}{\ddot c_z+g}\,\ddot c^{xy}+\frac{1}{m(\ddot c_z+g)}\,\hat z\times\dot k\;}$$

**物理分解**（便于可视化，必须分别算出）：

$$p_{\rm CoP}^{xy}=\underbrace{c^{xy}}_{\text{CoM 投影}}\;\underbrace{-\frac{z_c}{\ddot c_z+g}\ddot c^{xy}}_{\text{惯性项 (CoM 加速度)}}\;+\underbrace{\frac{1}{m(\ddot c_z+g)}\hat z\times\dot k}_{\text{角动量项 (AM)}}$$

- **变高**：$z_c,\ddot c_z$ 用**瞬时实际值**，不假设恒高。深蹲/单腿/踢腿时 CoM 高度变化，此式仍准。
- **退化**（仅校验用）：令 $\dot k{=}0$ 且 $\ddot c_z{\approx}0$ ⟹ LIPM/ZMP 式 $p_{\rm CoP}^{xy}\approx c^{xy}-\tfrac{z_c}{g}\ddot c^{xy}$。
- **这是预测式**：只需运动学量 $(c,\ddot c,\dot k)$，**不需要接触力** ⟹ 可用于**参考轨迹**（sim 无接触也能算）。
- **与接触式 CoP 的关系**：当存在平足接触时，理论上 $p_{\rm CoP}^{\rm (AM)} \equiv p_{\rm CoP}^{\rm (contact)}$（同一物理量两种算法）；数值上因模型/有限差分有小误差。**CoP-AM 的价值 = 接触发生前/无接触时仍能预测 + 把角动量贡献分离出来**。

### 1.3 角动量 $k$（绕质心）

定义：$k=\sum_i\big[I_i\omega_i+m_i(r_i-c)\times(v_i-\dot c)\big]$（$r_i,v_i,\omega_i,I_i,m_i$ 为第 $i$ 连杆的 CoM 位/速/角速/世界系惯性/质量）。

**本仓库捷径（推荐）**：已用 pinocchio。直接取质心动量的角分量：

```python
# pinocchio 已在 PinocchioCom 中接好；同样 model/data 即可：
pin.computeCentroidalMomentum(pin_model, pin_data)   # 或 centroidalMomentum
h = pin_data.hg                      # 6D 质心动量 [l(线), k(角)]
l = h.linear                         # = m·ċ
k = h.angular                        # 绕 CoM 的角动量  ← 用这个
```

> 注意参考点必须是 **CoM**（不是世界原点/基座）。pinocchio 的 centroidal momentum **默认 about CoM**，符合要求。

### 1.4 $\ddot c$ 与 $\dot k$（有限差分）

帧率 `fps`：

$$\ddot c[t]\approx(c[t{+}1]-2c[t]+c[t{-}1])\cdot fps^{2},\qquad \dot k[t]\approx(k[t{+}1]-k[t{-}1])\cdot\tfrac{fps}{2}$$

- 有限差分有噪声：建议对 $c(t)$、$k(t)$ **先轻量平滑**（如 Savitzky–Golay / 移动均值）再差分。
- 进阶（可选）：用 pinocchio 解析导数 `computeCentroidalDynamicsDerivatives` 取 $\dot k$，免差分噪声。

### 1.5 支撑多边形 $\mathcal P$（已有 `support_polygon`，复用）

- 脚 site：`right_foot_site` / `left_foot_site`（见 `models/jingchu01_..._sim2sim.xml`）。
- 脚底 box 尺寸（来自 XML `*_foot_collision_box size="0.135 0.055 0.015"`）⟹ **`half_length=0.135`，`half_width=0.055`**（脚约 27cm×11cm）。传给 `support_polygon(..., half_length, half_width, height_threshold)`。
- stance 判定：脚 site 高 ≤ `min_z + height_threshold`（默认 0.05）视为接地。
- $\mathcal P$ = 当前**接地脚**底面角点的 xy 凸包。单脚支撑 ⟹ 单脚矩形；双脚 ⟹ 双脚凸包。**CoP 必须在当前 $\mathcal P$ 内。**

### 1.6 出界判定（成败信号）

对凸多边形 $\mathcal P$（xy，逆时针顶点），点 $p$ 的**有符号距离**：

$$d(\mathcal P,p)=\begin{cases}\min_{\text{边 }e}\ n_e\cdot(p-e_{\rm ref}) & p\in\mathcal P\ (\ge0)\\ \max_{\text{边 }e}\ n_e\cdot(p-e_{\rm ref}) & p\notin\mathcal P\ (<0)\end{cases}$$

（$n_e$ 边外法向；内正外负。）**画 $d(t)$ 时序曲线**：失败段 = $d(t)<0$（持续越久/越深越危险）。这是"成败 ↔ 出界"的直接量化对照。

---

## 2. 在本仓库怎么实现（具体接入点）

### 2.1 复用（不要重写）

`scripts/replay_mcap_com_support.py` 已有：
- `compute_cop(model,data,support_body_ids) -> [x,y,z]`：**接触式 CoP 真值**（wrench→CoP，公式已核验正确：`x=(zF_x−M_y)/F_z, y=(M_x+zF_y)/F_z`）。
- `support_polygon(...)`、`support_foot_body_ids(...)`、`PinocchioCom`、`add_overlay/add_marker/add_segment`。

### 2.2 新增函数（建议签名）

```python
def compute_cop_am(c_prev, c_cur, c_next, k_prev, k_cur, k_next,
                   fps, mass, g=9.81):
    """CoP-AM (M-7)，纯运动学预测式。
    返回 (p_cop_xy[2], com_term[2], am_term[2])，后两项用于分解显示。"""
    zc      = c_cur[2]
    ddc     = (c_next - 2*c_cur + c_prev) * (fps**2)      # c̈
    kdot    = (k_next - k_prev) * (0.5*fps)                # k̇
    denom_z = ddc[2] + g
    if abs(denom_z) < 1e-6:
        denom_z = 1e-6 * (1 if denom_z>=0 else -1)
    com_term = -(zc/denom_z) * ddc[:2]
    am_term  = (1.0/(mass*denom_z)) * np.cross([0,0,1], kdot)[:2]
    p_cop    = c_cur[:2] + com_term + am_term
    return p_cop, com_term, am_term

def signed_distance(polygon_xy, p_xy):
    """凸多边形（逆时针）到点的有符号距离，内正外负。"""
    # n_e·(p - e_ref)，逐边；在内取 min，在外取 max（负）。
    ...
```

> $k$ 的获取：在 `PinocchioCom.compute(...)` 旁加一个 `compute_angular_momentum(...)` 返回 `pin_data.hg.angular`；或扩展 `compute` 同时返回 `(com, k)`。

### 2.3 扩展 `add_overlay`（同时画两个 CoP + 出界着色）

新增可视化元素（见 §3 规范）：
- 接触式 CoP（真值，已有，黄）；
- CoP-AM（预测，**按 $d$ 着色**：$d\ge0$ 绿 / $d<0$ 红）+ 轨迹尾迹；
- 两者连线（灰短线，看误差）；
- AM 贡献向量箭头（从 $c^{xy}$ 指向 $c^{xy}{+}\text{am\_term}$，可选）；
- $d<0$ 时支撑多边形闪红。

### 2.4 两条运行路径（都支持，意义不同）

| 路径 | 输入 | 用哪个 CoP | 回答什么 |
|---|---|---|---|
| **A. live sim2sim 真值** | 跑实际 `mj_step` 的 sim2sim 记录（contact 合法） | **接触式 `compute_cop`（真值）+ CoP-AM（预测）对照** | **"失败确实因 CoP 出界"**（真值说话） |
| **B. 参考轨迹预测** | 回放**重定向参考**（mcap），`set qpos` 不 `mj_step` | **只用 CoP-AM**（无 contact 求解） | **训练前预测哪些段危险** |

> ⚠️ **关键坑**：纯运动学回放（`set qpos` 不 `mj_step`）下，`compute_cop` **不可靠**（contact 未求解、力无意义）⟹ **路径 B 只用 CoP-AM**。要拿接触式真值，必须走路径 A（有真实动力学步）。

---

## 3. 可视化规范（画什么）

| 元素 | 颜色/形状 | 含义 |
|---|---|---|
| CoM 投影 | 蓝 sphere | 质心落地 |
| 支撑多边形 $\mathcal P$ | 绿线框；$d<0$ 时**变红** | 可行边界 |
| 接触式 CoP（真值） | 黄 sphere | 地面真值（仅路径 A） |
| CoP-AM（预测） | sphere：$d\ge0$ **绿** / $d<0$ **红** + 尾迹 | 预测式 + 出界警示 |
| 真值–预测连线 | 灰短线 | 模型/差分误差（小=准） |
| AM 贡献向量 | 紫箭头 | 角动量把 CoP 推多远 |
| HUD/打印 | 文本 | `d(t)=..`，$d<0$ 标 `OUT` |

---

## 4. 验收 / sanity check（防实现错）

1. **静止站立**：$\dot k{\approx}0,\ddot c{\approx}0$ ⟹ $p_{\rm CoP}^{\rm AM}\approx c^{xy}$（落在支撑中心）。✓
2. **准静态慢走**：CoP-AM 应 ≈ 接触式 CoP（误差 < ~2 cm）。差很大 ⟹ 有限差分噪声 / 单位 / 公式错。
3. **量纲检查**：$\tfrac{z_c}{\ddot c_z+g}$ 量纲 $=\mathrm{s^2}$，乘 $\ddot c$（m/s²）= m ✓；$\dot k$ 量纲 $=\mathrm{N\cdot m}$，除以 $m(\ddot c_z+g)$（N）= m ✓。
4. **退化校验**：强制 $\ddot c_z{\equiv}0$ ⟹ 与 LIPM/ZMP 式一致。
5. **失败帧吻合**：$d(t)<0$ 的时段应与实际摔倒/失衡时刻吻合。

---

## 5. 常见坑

- **有限差分噪声**：$\ddot c$ 差分两次误差大 ⟹ 预平滑 / 降帧 / 用解析导数。
- **角动量参考点必须是 CoM**：pinocchio `hg.angular` 已是 about CoM ✓；若手算务必绕 $c$，不能绕世界原点。
- **变高**：别代常数 $z_c$；$z_c,\ddot c_z$ 都用瞬时值。
- **飞行相（飞踢腾空）**：无接触 ⟹ 接触式 CoP 无定义；此时只显示 CoP-AM（作"假想 CoP"参考），或切到 capture point / DCM 可视化。CoP-AM 在飞行也不严格有效（前提是有 wrench），仅供定性参考。
- **脚 box 尺寸**：`half_length/width=0.135/0.055` 来自当前 XML；**换机器人/配置必须更新**（查对应 `*_foot_collision_box size`）。
- **m 总质量**：用 `model.body_mass.sum()`（含所有 body），别漏 link。

---

## 6. 交付物

1. 扩展 `replay_mcap_com_support.py`（或新脚本）：加 `--cop-am` 开关，同时画 CoP-AM + 接触 CoP + 多边形 + 出界着色 + `d(t)` HUD；支持两种输入（参考 mcap / sim 记录 mcap）。
2. 示例：对一条会摔的动作，给出 $d(t)$ 曲线截图 + 失败帧 CoP 出界的 viewer 截图，证"失败 ↔ CoP 出界"。
3. 对参考轨迹（路径 B）输出一份"危险段"清单（哪些时间窗 $d(t)<0$），供训练前筛选动作。

---

## 附：与上游研究文档的对应

- 本公式即研究路线 D 的 **CoP–AM 恒等式 (M-7)**（见 `math-derivation-and-code-mapping.md` §2）。
- 支撑多边形 + 出界判定，即该路线"可行性投影"的**可行集边界** $\mathcal P$ 的可视化体现。
- 此可视化用于支撑论文/汇报的头条对照实验：**"裸 mimic 失败帧的 CoP 出支撑多边形；可行感知方法把 CoP 拉回界内"**。
