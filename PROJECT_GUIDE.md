# 反恐电控工程导览与调参索引

更新：2026-10-04。当前共用工作工程为 `C:/Users/15119/gongkesai/gongke`（GitHub `zjj20040618-sudo/gongke`）；不要与桌面交接包或旧 `jiejie` 仓库混编。Keil 从 [MDK-ARM/jiejie.uvprojx](MDK-ARM/jiejie.uvprojx) 打开，工程名不代表仍使用旧仓库。

视觉接收侧新增二进制适配；完整接口与未完成项见 [VISION_INTEGRATION.md](VISION_INTEGRATION.md)。下面的 2026-09-30 编译/ASCII 检查条目是历史记录，不是新协议的整机验收。

这是一张**找代码的地图**，不是“全部参数已经调好”的证明。`TODO`、`0` 和标注为“种子”的值都要按实车测试填写。当前正式整场有配置闸门，不能因工程能编译就直接上车跑。

## 2026年10月4日 无机械臂单向联调

当前源码固件号 `20261004-VISION-DIAG33`。新增模式33只静止收数，操作见 [VISION_TEAM_HANDOFF.md](VISION_TEAM_HANDOFF.md)：33→g请求QR，合法三任务后自动切OBJECT，每秒回传VD33；再g或a/0停止，仍选择33供下轮g。运行中拒绝运动/参数写命令，串口回调只缓存，打印/TX在DefaultTask。已有上电舵机PWM不等于舵机断电；本入口不发机构动作。

31仍只走原10段路线；20、22、30及31的整数轮速和旧调试动作不改。下面10月3日及先前32的HEX都是旧候选，不作为本轮已烧录证明。

32流程见 [App/mission_trial.c](App/mission_trial.c)，原距离和角度取 [App/mission_trial_plan.c](App/mission_trial_plan.c)。左移500后停稳、请求QR，等待本轮完整合法三任务QR才继续，不固定等待10秒：后600→左95→前750整条越障路→左730→前830→右85→任务区两段→右85→救援走廊2125。任务区原2450拆为“入口转角→桶”的第一段 `b1d`（待用户实测）与“桶→下一转角”的第二段 `2450-b1d`；不是两段完成后再走2450。普通行走v100，像素对位仍限速12～80mm/s。750含物理障碍道路，不另外加一次越障距离；不插入尚未定时的倒退靠障碍方案。

先按QR选球：对齐→停10秒→顺时针180→桶对齐→停10秒→顺时针180回来；此时以桶停车节点重新建立第二段账本，再单向经过反恐区。`b1d`是桶参考位置，不是球搜索截止线；抽签后的指定球可能在桶沿路后方，球搜索仍受原完整2450走廊预算限制，不在b1d处提前结束。QR指定靶出现时刹停、对位、激光亮2秒；人质区同样停下对位，再停10秒占位。靶/人质在对位开始停稳后记录道路进度；若对位造成净后退，任务完成后 `TARGET_RETURN/HOSTAGE_RETURN` 前进补回原停车进度，再走新计算的余量。对位净前进不倒退回原点；急停不触发补回。三个10秒仅替代机械动作，32不调用步进或舵机。相机、机械臂朝车身左侧，画面cx修正使用车身前后轴；纵深站距仍未标定。

2026-10-04用户纠正：球的位置顺序由抽签决定，不固定蓝、绿、红。按QR颜色与实际视觉坐标对位，不能按颜色预设停车位置或180度后左移/不动/右移。整条2450搜索走完仍未识别到球时，标示 `BUCKET_ANCHOR_MISSING`，不创建假桶节点、不再追加一整段第二段，也不虚构任务hit。

道路账本每1ms按连续IMU航向投影编码器增量，停车、对位、制动和转身期间也计入。桶侧180姿态位移按原路方向带符号计入第一段；桶节点只重建一次第二段账本。靶/人质补回过程中继续积分原账本，不另给余程加“倒退量”，否则重复补偿。普通移动保持路线原航向，32平移/对位使用小数轮速；转身保留成功180的整数轮速保持参数。前进的RAM `fff`默认0.0125，不套给倒退、左移或像素对位。上述仍是轮式里程估计，不证明真实横向位置或投影不出线；本轮未改轮位、轮速PI或航向参数。

启动前在BOOT设置一次RAM：`b1d数值`填入口转角到桶的第一段实测命令mm（1～2449，默认0未确认），第二段自动算2450减第一段；`vsg1/vsg2`指定cx偏大时向车头/车尾修正；`bcx/tcx/hcx/kcx数值`指定球、靶、人质、桶的实测目标cx（命令范围0～479，但必须在真实OBJECT画幅内）。未提供第一段或工作点时32不启动，不自动猜值；没有机械臂时可标联调参考点，但不能当作最终抓取点。`trial`只读查看。发`32`，再`g`开跑；g自动回传参数，每秒报告阶段/进度。再g或a/0中止并关激光，不续跑或自动补回；联调整链终端后重新测试须重启。

视觉需部署完整9541十类App，见 [VISION_CONTROL_PROTOCOL.md](VISION_CONTROL_PROTOCOL.md)。电控按QR筛选类别/标签，新增0腰鼓/2圆锥/9桶接收映射；训练别名与真实实物仍待联调。OBJECT实际画幅读取模型输入，32工作点旧命令范围仍0～479，现场先用33核对画幅，再填写兼容的参考点，不沿用旧480×320假设。新入口无往返补扫或桶扫描归位；`ROUTE_END`不等于任务完成，`hits`只记录本轮视觉/占位链：球桶1、靶激光2、人质4，均满足为7。

实际验证和候选HEX位置见 [CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)。未烧录、未做实车联调，不能以主机模拟或编译替代现场验收。

### 当前直行控制链

`v/d → 道路速度vx/vy + IMU航向修正w → 麦轮逆解四轮rpm → 编码器每轮速度PI → PWM`。速度v单位mm/s、d为轮式换算mm；轮速环在 [App/control.c](App/control.c) 每1ms运行，默认kp=0.05、ki=0.004、kd=0；航向是独立P环，默认ykp=0.3，实际RAM值以蓝牙回传为准。32在 [App/mission_trial.c](App/mission_trial.c) 保存跨段航向目标，并把道路方向换算到车体方向；15/17/31仍为旧整数轮速入口，32保留小数修正。

前进fff=0.0125在v100时相当于叠加向左1.25mm/s，是前馈而非真实侧偏位置闭环。acc/dec默认0，仍是恒速到点刹车；停车后未主动二次纠角。因此“车头保持正”不等于地面轨迹没有右串，现有编码器/IMU不能独立测到全部滑移。这轮不改上述参数，后续走直调试单独处理。

## 2026-10-03 步进与夹爪队友交接

接线、24～29蓝牙操作、测量顺序和一次性回包表见 [ACTUATOR_TEAMMATE_HANDOFF.md](ACTUATOR_TEAMMATE_HANDOFF.md)。队友按它辨认两轴方向，标定STEP与实体位移、夹爪脉宽、抓球放桶/抓人质两套固定动作。新增“出发前不外伸、出发后展开”要区分收拢S与任务起点O；当前上电1400µs和缺少原点反馈不保证该要求成立，展开节点及参数仍待确认。本次仅文档准备，未修改固件或发布新HEX。

## 2026-10-03 左95/右85、d830（当前）

当前用户定值为左95°、右85°；确认左移730后的原直走780加50改830。当前源码固件号 `20261003-ROUTE31-L95R85-D830`，蓝牙20=+85、30=-95，22=+180不变。两侧目标在 `App/test_config.h::T_TURN_RIGHT_TARGET_DEG/T_TURN_LEFT_TARGET_DEG` 分别配置；起跑comp=-5/+5以及SEQ/REC实际目标同步，不共用一个95常量。

31完整配方：左500→后600→左95→前750→左730→前830→右85→前2450→右85→前2125，仍全v100、无QR/任务。`mission.c::R_CROSS_EXIT_FWD_MM`也同步830；正式任务转角仍名义90、全场闸门未打开。首段左移右歪3°尚待蓝牙日志定位，不通过改变转角掩盖；横移PID/前馈、轮位/极性未改。独立候选/构建见 [CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)，下面双95方案被新口径替代，未发布HEX。

已编译的本版工程为 `C:/Users/15119/gongkesai/backups/route31_l95r85_d830_candidate_20261003_092529/MDK-ARM/jiejie.uvprojx`，HEX在该候选 `MDK-ARM/jiejie/jiejie.hex`，SHA256 `A629E85D77DB672401BD82BA275CAB9D296CB2DE533A52A73F840FF312037DA4`。主机回归及Keil全量0 Error / 0 Warning通过，相关源码/AXF标识核对一致。常用工程路径HEX仍是旧095B...版本，不是本版；本轮未烧录/推送，旧候选保留。

## 2026-10-03 左移歪头诊断与双95°（中途被替代）

用户确认第一段左移d500后车头右偏约3°，要求转角候选改95。源码ID `20261003-ROUTE31-TURN95`，`App/test_config.h::T_TURN90_COMP_DEG=5`：模式20+95、30-95，31三处转身同步，180、d600/v100和其余数值不动；起跑/SEQ/REC均显示实际目标。这是相对于名义90的总5°补偿，不是92+5；正式任务不套本候选。

左移右歪并非新fff混入：模式17 v100 ff=0，航向/轮速/轮位保持旧版。纠偏符号按源码向左，实际效果/运动中还是停车后才偏要看本轮蓝牙包。旧到距后不继续摆正而下段清零继承偏角，本轮只记录诊断，未无依据改PID或增加自动动作；95°不能修复第一段左移本身。独立工程/HEX及构建结果见 [CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)，下节为前版历史。

## 2026-10-03 路线转弯92°（前一阶段）

用户要求当前90°转弯增加2°：`App/test_config.h::T_TURN90_COMP_DEG=2`，模式20=+92、30=-92，31第3/7/9步分别左92/右92/右92。转向目标、惯性回调、角限和蓝牙角度显示统一按该命令；名义90几何及正式未放行任务不套候选值，22始终180不变。固件ID `20261003-ROUTE31-TURN92`，第二段d600/v100、其余距离/速度、fff补偿不变。新版构建交付见 [CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)；下文均为前版记录，不升级成实体角度验收。

当前独立工程 `C:/Users/15119/gongkesai/backups/route31_turn92_candidate_20261003_090109/MDK-ARM/jiejie.uvprojx`，对应HEX在该候选 `MDK-ARM/jiejie/jiejie.hex`；Keil全量0 Error / 0 Warning，SHA256 `7060091098EF953451AB904E6EA7336783C6AE5998D5AFC75F611DAEA2B531B2`。主机回归、候选与源码哈希一致性和AXF版本检查通过；未自动烧录/推送，旧版保留。

## 2026-10-03 第二段d600（前一阶段）

用户把第二段改为沿车尾后退d600，v100不变。源码 `App/mission.c::QR_BACK_MM=600`、`App/route_test_plan.h` 第二项600同步，固件ID `20261003-ROUTE31-BACK600`；其它9步、前进小补偿和共用转向保持参数不动。下文旧550及候选HEX为前版记录，新编译工程/HEX见 [CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)。

补偿方向已按当前轮位/IK核查：直走的左向小横移是对角“左后+右前”略快，不是整侧右轮加快；整侧右轮略快会让车头左转。保持原fff正值向左的定义，实体接线改变时仍需核对轮位。

本次d600独立工程 `C:/Users/15119/gongkesai/backups/route31_back600_candidate_20261003_085030/MDK-ARM/jiejie.uvprojx`；该候选HEX全量0 Error / 0 Warning，SHA256 `A569390BB8E63CBE5886A096E12FC0B75350383B316ACD1FE4CC04A04D1C61F5`，已核对AXF标识及源码。常用工程HEX目前仍是前一补偿版095B...而非本次d600，务必区分；未自动烧录/推送，下面旧DA4...输出状态是当时检查记录。

## 2026-10-03 路线31小补偿与共用转向保持（前一阶段）

源码固件号 `20261003-ROUTE31-FFF-TURN`。在下面初版31配方上只添加前进小补偿：`App/test_config.h::T_FORWARD_FF_SEED=0.0125`，模式15 v100/v200向车头左侧给额定vy；31的四段前进都启用，后退和横移不套。蓝牙 `fff0` 关闭、`fff0.0125` 恢复，上电恢复默认，无Flash save；开跑自动参数包包含fff。整数轮速量化使v100实下发29/28/29/28，不能把名义12.5mm/m当实体保证。正式直走暂不套用此候选。

转向实际核对结果：20/22/30原本已经同算法，同KP/限速/容差/超调回调/静置判据；统一到 `App/turn_profile.h::TURN_HOLD_*` 并将±90最长修正预算与成功180同为12s，180参数保持。目标仍±90和+180，不用猜测92°补偿；完成仍须容差≤0.3°且静置700ms稳定。正式-90/±180旧分支本轮未改；下面历史记载不代表已经全面集成。

主机补偿与同误差转向回放、完整10段路线和120组停止检查通过；新独立工程/HEX及最终构建结果登记在 [CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)。无需重新输入每段v/d，烧录新版后仍 `31→g` 起跑、再g取消，实体效果等本次试跑。

新版独立工程 `C:/Users/15119/gongkesai/backups/route31_ff_turn_candidate_20261003_083831/MDK-ARM/jiejie.uvprojx`，HEX在该候选的 `MDK-ARM/jiejie/jiejie.hex`；Keil全量0 Error / 0 Warning，SHA256 `095B1A0C2E1627DB964273D0E87AEC865446FC779F5EA82867F21352D93A6011`。旧常用HEX和下节初版31候选均保留，不自动烧录/推送。

## 2026-10-03 只走路线模式31（初版，已由上节更新）

本轮随后新增“只走路线”模式31，当前源码ID `20261003-BASE-ROUTE31`。蓝牙 `31→g` 顺序执行 `App/route_test_plan.h` 的10步配方：左500→后550→左90→前750（含整条障碍路）→左730→前780→右90→前2450→右90→前2125，所有平移v100。复用既有单段速度/航向/定距/转向控制；节点准备单独非阻塞，g/a/0取消全链，不续跑/倒回。不会扫码、开激光、执行任务、爪或步进，亦未开启正式闸门。750/2450/2125整段记录与正式任务间各子段不能混算；正式子段仍待联调。开跑自动发diag/param，进度SEQ、结果REC、`route`查询都走ASCII蓝牙。全套主机回归及120组停止检查通过，后续构建/烧录以实际结果为准。

独立工程：`C:/Users/15119/gongkesai/backups/route31_candidate_20261003_082509/MDK-ARM/jiejie.uvprojx`；对应已编HEX在其 `MDK-ARM/jiejie/jiejie.hex`，日志 `MDK-ARM/rebuild_route31_candidate.txt` 全量0 Error / 0 Warning，SHA256 `4EB82DF0D270E0FAFFD1014552A5813EB5CFFB59D2346C81278277B333B64F66`。已核验AXF固件标识与配方标签；常用工程路径HEX仍是旧稳定版 `DA4DA13A6B8466FF9DCC16E462DB29362896CCCD7A7BDE14F158EE1DFADBB109`，不要在它上面盲发31。未自动烧录/推送，整段实车验收待用户这次试跑；停止后要人工回到起点再重选31，g不恢复中断位置。

## 2026-10-03 稳定回滚基线与左90°候选（前一阶段）

用户已选择完整恢复 `aadf028`（2026-10-02 02:16）并把编码器接回旧位置；不混入10月3日的大改。当前仅新增实测登记 `QR_START_LEFT_MM=500`、`QR_STR_V_MMS=100`、`QR_BACK_MM=550`、`QR_BACK_V_MMS=100`、`R_CROSS_EXIT_LEFT_MM=730`、`R_CROSS_EXIT_FWD_MM=780`、普通路线直走/横移速度100，以及 `test.c/test_config.h` 的模式30左90°入口。速度100的实测范围限本轮已报路段，后续任务间同速仍待验证；`d750` 的越障路起止口径待确认，尚未拆填接近/越障/剩余距离。`turn_profile.h`、轮位/极性、速度环和直走控制律不变；没有重开自动编码器停机保护，也没有推送。本次数值登记后主机回归通过，未为新增数值重编HEX；下面候选快照只有当时的500/550和左90°入口，不含随后登记的730/780。

当前源码固件号 `20261003-BASE-LEFT90`。模式30目标-90°，镜像复用20控制并反向微调超转；等待阶段g取消未来回调。主机回归通过，左转实体效果仍待测；正式负90°函数仍是旧通用控制，未把候选参数套进整场。测试方式是 **候选烧录后** `30 → g`，运行中g停止，到位后g清态但不转回；r3仍只前进、不左转或越障。

为了不打断用户当前直走测试，候选工程在 `C:/Users/15119/gongkesai/backups/left90_candidate_20261003_075345` 单独编译：`MDK-ARM/rebuild_left90_candidate.txt` 全量0 Error / 0 Warning；候选 `MDK-ARM/jiejie/jiejie.hex` SHA256 `7DA5235688CF3DEE7229B72B287C32D0357504692E01778C47E952B81EF5146A`。常用工程 `MDK-ARM/jiejie/jiejie.hex` 没覆盖，仍是回滚基线SHA256 `DA4DA13A6B8466FF9DCC16E462DB29362896CCCD7A7BDE14F158EE1DFADBB109`，不能在旧HEX上使用30。未烧录，四个正式标定闸门仍0。下面各节是历史阶段记录，以本节和 [当前逐段实测](CONTROL_TUNING_TODO.md) 为准。

## 2026-10-01 出发方向纠正（本地未发布）

当时机械臂朝车身左侧已确认，相机同朝左侧随后于2026-10-02由用户确认；激光是否同侧/共轴仍须核对。前三段按车体系为：**出发左平移 → 倒退 → 左自转90°后前进，准备越障**。该轮只改 `mission.c` 出发段和 QR 端点补扫、`test.c` 的 `r1/r2/r3`；本轮最新视觉换轴见下节，旧“仅救援右90°和排爆两次180°”不再是全程转身次数的定论。

正式第一、二段距离在 `QR_START_LEFT_MM` / `QR_BACK_MM`，速度在 `QR_STR_V_MMS` / `QR_BACK_V_MMS`；第三段前 `route_pre_cross_turn()` 调用 `step_nav_leg(-90,0,0,0)`，然后沿新车头前进 `R_PRE_CROSS_FWD_MM` 再越障。有效三任务 QR 仍是越障门槛；补扫沿倒退段前后往返，发现码后回到已知第三段起点，不中途转身。当前全部标定闸门和未测参数仍关闭/置0，不复用旧方向的尺量值。

台架 `r1`=17左移/v300，`r2`=16后退/v80，`r3`=15前进/v80；三者清空距离，必须重新设 `d`，只测单段。`r3` 不替用户完成左转；20仍仅右90°，新增左90°用通用 `ROT_*` 控制，尚无独立台架入口或实车验收。当前源码固件号 `20261001-ROUTE-LEFT-BACK`，不是已烧录证明。出发端点回放和负90°首拍方向回归已通过，均属主机/合成验证，不证明实体路程、扫码视野或左转角度。

本轮全套 `tests/run_host_tests.ps1` 通过；新增 `mission_departure_route_test.c` 直接包含实际 `mission.c`，检查四种扫码发现时机均回到已知端点，以及中止、零参数阻断和左转调用的成功/失败/中止。Keil 全量重编日志为 `MDK-ARM/rebuild_route_left_back_2026-10-01.txt`，0 Error / 0 Warning；HEX SHA-256 为 `1CE9B6673D38F79C6573440222FB636ABC31901C66D2C2924113A8D4A87C7FFC`。配置闸门仍关闭，map 裁掉正式 `step_rotate_deg`；编译证据不表示本 HEX 可运行新增左转整场。未推送、未烧录、未实车测试。

## 2026-10-02 任务区前右转节点（本地未发布）

用户进一步确认完整顺序：**走完第三段（障碍只是其中一部分）→ 左平移 → 直走 → 原地右转90° → 沿新车头直走进入任务区**。第三段仍分为 `R_PRE_CROSS_FWD_MM` 接近、姿态判据越障、`R_CROSS_REST_FWD_MM` 剩余直线，不能刚过坎就左移。旧 `R_EOD_ENTRY_RIGHT_MM` 右横移已替换为 `R_EOD_ENTRY_RIGHT_TURN_DEG=90`，调用 `step_nav_leg(90,0,0,0)` 先停稳再自转，复用现有右90°分支；随后 `route_straight(R_EOD_ENTRY_FWD_MM,ROUTE_FWD_V_MMS)` 直走进区。该距离仍置0并纳入启动闸门，待实测，不从旧横移距离照抄。侧装相机对位、任务链及后半场仍待确认，参数闸门继续关闭。

当前源码固件号为 `20261002-ROUTE-EOD-FWD`，上节固件号和HEX是前一轮历史记录。主机回归检查“障碍后剩余直线→左移→直走→右90°→直走进区”源码顺序及新距离的零值闸门，并补实际 `step_rotate_deg(+90)` 首拍正方向/中止检查；这些不证明整段实体路线或带载转角合格。

同日先前右转节点版的重编日志为 `MDK-ARM/rebuild_route_eod_right90_2026-10-02.txt`，HEX SHA-256 为 `2B470D7854837F87DD22F19105183ECB6969B424430A8012727C3049523B946E`。补直走进区后，全套主机回归再次通过，Keil 全量重编日志 `MDK-ARM/rebuild_route_eod_forward_2026-10-02.txt` 为 0 Error / 0 Warning，最新HEX SHA-256 为 `E32CAB336E327CFA8DE6AC2BA4B9C2E9BD57BC8553E06BE2D3C8AF0CB708167D`。正式转向仍因配置闸门由链接器裁掉，不能用当前HEX验收这段整场路线。未提交推送、未烧录、未实车复测。

## 2026-10-02 任务调用对应审计（本地未发布）

用户已确认相机和机械臂同朝车身左侧，激光仍须确认。不全局替换前进/横移：固定抓取的下降/齿条伸出/抬升是机构轴，不因整套安装朝向改变就翻转步进DIR；这些动作及双轴回程本轮不改。

| 对应调用 | 旧朝前相机假设 | 当前左侧相机适配状态 |
| --- | --- | --- |
| `steps.c::step_align` 像素cx对位 | 速度下发到车身横移轴，锁前后里程 | 已换为前后速度、锁侧向里程；`VISION_CX_FWD_SIGN=0`未标定拒绝驱动，±1须实测 |
| 同函数像素高估距 | `step_straight(err,...)` 靠近 | 已通过`align_depth_move`换为左平移靠近/右平移离开；站距/工作点仍需标定 |
| `steps.c::step_sweep` 找球/桶/靶/人质 | 横向里程端点、横移往返 | 已换前后里程端点/速度并锁侧向串动，`SWEEP_FWD_MMS`及各类范围仍待实测 |
| `task_eod.c` 放桶后回扫描起点 | 横向里程、`step_return_lateral_odo` | 已同步为`motion_odo_mm`及`step_return_forward_odo`，先回第一次转身后的同轴基准再180°转回 |
| `mission.c` 任务入口基准、离区路线 | 排爆/反恐/救援均记横向里程 | 三处入口和离区已用前后轴；救援入口基准在救援前右90°之后另记，不混用转身前里程，距离仍待测 |

独立于安装方向的VC-06已修：像素误差绝对值≤8才进入连续5帧对准判定；容差外最小修正速度12 mm/s、上限80 mm/s，保留原方向符号/轴，不再把未达标误差清成0。`tests/align_boundary_test.c` 将合成帧送入实际 `step_align`，11例覆盖0、±8、±9、±19、±20、±140并检查实际速度调用；同时全套主机回归通过。当前固件号 `20261002-ALIGN-TOL-FIX`，Keil全量日志 `MDK-ARM/rebuild_align_tolerance_2026-10-02.txt` 为0 Error / 0 Warning，HEX SHA-256 `41DBBF34054C3D5AFA158DEB0B354D99D64DB2DEC6FF57B80F1C913E41E61C14`。正式对位因标定闸门关闭仍由链接器裁掉，编译不证明实体收敛。未推送、未烧录；现场须验证安装、画面方向、静摩擦和过冲，VC-06仍是待联调而非已验收。

上段是死区修复阶段的历史证据；本轮确认安装后已完成上述换轴，最新固件号为`20261002-VISION-LEFT-AXIS`。正式像素符号默认0；主机单独以±1测试配置各检查16例（11个对位边界、2个靠近/离开、2个返回方向、1个扫描到远端后反向发现目标），另检查生产0符号拒绝驱动；全套主机回归通过。Keil全量日志`MDK-ARM/rebuild_vision_left_axis_2026-10-02.txt`为0 Error / 0 Warning，HEX SHA-256 `62AE9F6093AF288BE8509EE9EB48D41079222D33C0FA51DC029558079F0D6D01`。测试配置不写回正式参数；工作点、扫描范围、实际像素方向、激光以及后半场路线仍待确认/实測。未提交推送、未烧录/实车验收。

## 2026-10-02 排爆后直走去反恐（本地未发布，前一阶段）

用户确认：**排爆抓放结束、车身转回原朝向后，沿车头直走去反恐区**。只调整这段，不提前改反恐后的路线。`R_EOD_TO_ANTI_RIGHT_MM` 已改为 `R_EOD_TO_ANTI_FWD_MM=0`，定义为排爆带入口到反恐带入口的前后里程差，零值仍阻断整场启动。

进入排爆时以 `eod_entry_fwd_odo=motion_odo_mm()` 记录前后基准；任务结束后调用 `route_straight_to(入口基准+整段距离,ROUTE_FWD_V_MMS)`。先停稳再计算余量，扣除找球/对位已走的前后距离，避免从抓球停点重复走整段。若已越过目标则按余量反向返回；容差0.5mm是轮式里程判据，不是实体定位精度。放桶扫描回程先回第一次180°之后的基准、再转回来；旋转误差、打滑等仍需实测，累计车体系里程不是地图绝对坐标。

当前源码固件号 `20261002-EOD-ANTI-FWD`；前文固件号/HEX均为历史阶段记录。全套主机回归通过，新增6例覆盖扫描位移、越过目标、已到目标、停稳余动、中止及执行失败；源码契约检查前后入口基准及任务顺序。Keil全量日志 `MDK-ARM/rebuild_eod_anti_forward_2026-10-02.txt` 为0 Error / 0 Warning，HEX SHA-256 `C8D2EE1FCADD4E7508E948E12AB54F46A08EA14564D4244CB233FC84FA05D50A`。四个标定闸门仍为0，正式路线未放行；没有新增实车数据，未提交推送、未烧录。反恐后去救援及救援后返回的方向、入口轴仍待逐项确认。

## 2026-10-02 反恐后直走到拐点、右90°（本地未发布，前一阶段）

用户确认：**反恐完成 → 继续沿车头直走到拐点 → 原地右转90°**。旧 `R_ANTI_UP_MM` 横移占位已改为 `R_ANTI_EXIT_FWD_MM=0`，表示反恐带入口到救援前拐点的前后里程差，不是从射击停点再走完整距离。进入反恐时以 `anti_entry_fwd_odo=motion_odo_mm()` 记基准，任务成功后使用 `route_straight_to` 扣除找靶/对位已走部分；直走失败不得转身，右转失败不得进入下一段。`R_RESCUE_RIGHT_TURN_DEG` 固定设计值90，并由启动检查拒绝其他值。

本轮只改到右转节点；转完后的救援进区、返回方向仍待用户逐项确认，旧占位不代表已审核。激光安装与工作点仍待联调，未改打靶流程、步进/舵机行程或轮位符号。车体系里程用于本段余量，不当全场坐标；转身后另建下一段基准。

当前源码固件号 `20261002-ANTI-EXIT-FWD`。全套主机回归通过：新增源码顺序检查覆盖反恐入口前后基准→任务成功→前后余量→右90°，以及距离零值和右转90°闸门；共享余量函数6例及右90°首拍方向/中止回归继续通过。Keil全量日志 `MDK-ARM/rebuild_anti_exit_forward_2026-10-02.txt` 为0 Error / 0 Warning，HEX SHA-256 `5E358DD3F30273A6894350232D93F489E34EBDD60513925C66D4A12E8A8F1700`。四个 `CAL_*_READY=0`、`BENCH_AUTO=0` 不变；距离仍待实测，正式整场未放行，未提交推送、未烧录或实车验证。

## 2026-10-02 救援直走进区、抓后直走返回（本地未发布，当前最新）

用户确认：**救援前右90° → 沿新车头直走进人质区 → 解救完成 → 不转身、继续直走到返回区**。旧横移占位改名为 `R_RESCUE_ENTRY_FWD_MM=0`（右转后的进区距离）与 `R_RESCUE_TO_HOME_FWD_MM=0`（救援入口至返回区的前后里程差），均由启动配置检查阻断未标定运行。入口 `route_straight` 使用直行速度；入口到位后另记 `rescue_entry_fwd_odo=motion_odo_mm()`，不沿用转身前的基准。

救援任务仍是扫描/对位→停稳→预降→伸齿条抓住→抬离平台，夹持不放、机构不自动复位。成功后 `route_straight_to` 从入口目标中扣除扫描/对位已走的前后里程，再直走返回，失败进入ABORT而不是假报DONE。已无后半场右横移调用，原 `route_strafe_to` 因无消费者删除；保留越障路末端的左横移。

目前用户逐段确认的整场动作：左平移→倒退→左90°→直走整条越障路（接近、过障碍、剩余直线）→左平移→直走→右90°→直走进入排爆→排爆抓放及两次180°→直走去反恐→反恐→直走到拐点→右90°→直走进救援→抓住抬升→继续直走返回。此为动作设计和源码对应，不是已跑通的实车结果。

当前源码固件号 `20261002-RESCUE-HOME-FWD`，前文各版构建为历史记录。全套主机回归通过，新增源码契约检查右90°→直走进区→新前后基准→救援成功→前后余量返回→DONE，检查抓后无新增转身/横移，以及两个距离的零值闸门；共享余量6例和机械任务4例继续通过。Keil全量日志 `MDK-ARM/rebuild_rescue_home_forward_2026-10-02.txt` 为0 Error / 0 Warning，HEX SHA-256 `6B6E7355DBC99DEB77F492B4DD834921DB1B301EF8DF5C9D66505D262306F3B3`。四个 `CAL_*_READY=0`、`BENCH_AUTO=0` 未改变，正式流程仍被配置闸门拦截；编译不证明当前HEX可执行整場任务。路线距离、相机符号/工作点/站距、激光安装、机械行程及带载效果仍待实测。本轮未提交推送、未烧录、未新增实车数据。

## 2026-10-02 本次提交版：左装路线与第二次g停止

本节是上述逐段本地核对的汇总，前文“未发布/未提交”是各阶段当时的记录。用户已确认流程并授权提交至共用 `main`；推送结果以本次实际 Git 提交编号为准。源码固件号为 `20261002-LEFT-ROUTE-GSTOP`，未烧录、无新增实机验收。路线与左侧相机换轴同本文件上文，不翻转机械轴DIR，不填猜测距离。

- 默认整场：首个 `g` 在标定齐全时申请启动，运行中再次 `g` 中止，`a` 保留为同一停止入口。启动请求已接受、MissionTask尚未苏醒时也能停止；中止后不自动回程、不清中止标志、不把第三次 `g` 当重启，重新运行整场需重新上电。
- 测试模式保留原启停口径：运行中 `g` 停止；旧5/6定时测试回程中也可再 `g` 取消回程。24/25仍保留停止后第三次 `g` 清态；26/27取消后续步数和自动回程；28/29取消后续动作但保持当前命令PWM，不能保证舵机立即机械停住。15～18等待制动完成后再操作，不自动倒车。
- 实际 `test.c` 蓝牙解析/状态机主机回放覆盖整场启动竞态、运行/终态停止、`a`、未标定闸门，以及单轮、定距、转向、IMU、步进、舵机和回程入口。全套 `tests/run_host_tests.ps1` 与11组 `tests/test_vision_binary_replay.py` 通过；主机编译仍报告旧 `cmd_reset` 未用局部变量警告，未顺手修改该无关代码。
- Keil全量重编 `MDK-ARM/rebuild_left_route_gstop_2026-10-02.txt`：0 Error / 0 Warning；HEX SHA-256：`DA4DA13A6B8466FF9DCC16E462DB29362896CCCD7A7BDE14F158EE1DFADBB109`。日志/HEX为本机构建输出，不提交仓库。四个 `CAL_*_READY=0`、`BENCH_AUTO=0` 和未测距离/速度仍保持；链接器可裁掉被配置闸门阻断的正式路径，编译不等于整场能跑。
- 相机自动切模式、桶/另外两类人质映射、真实串口、工作点、机械行程和实机路线仍看 `VISION_CONTROL_TODO.md` / `CONTROL_TUNING_TODO.md`，不能表述成仅剩填数值。

## 从哪儿开始读

```text
Src/main.c + Src/freertos.c  启动/任务调度
          ↓
App/robot.c               初始化、三路串口分流、蓝牙服务
          ├─ App/test.c               蓝牙台架测试（与整场互斥）
          └─ App/mission.c            正式整场状态机、QR 与路线
                    ├─ App/task_eod.c / task_anti.c / task_rescue.c
                    └─ App/steps.c + auto_steps.c  可复用动作/越障
                              ↓
             App/motion.c → control.c → board_pins.c（底盘）
             App/arm.c（齿条、丝杆、爪舵机）
             App/proto.c / imu.c（视觉帧、姿态输入）
```

第一次阅读建议顺序：`robot.c` → `mission.c` → 对应的 `task_*.c` → `steps.c` → `motion.c/control.c`；要调某个模块再看下表，不必先翻完 `test.c` 的全部模式。

## 每个自写文件负责什么

同名 `.h` 主要放对外接口/类型；算法和参数通常在 `.c`。本表覆盖 `App/` 的全部自写模块。

| 文件 | 职责 | 要找的参数/入口 |
| --- | --- | --- |
| [robot.c](App/robot.c) / [robot.h](App/robot.h) | 全系统初始化；USART2 视觉、USART3 蓝牙、UART4 IMU 接收分流；周期任务入口与诊断回传 | `FW_BUILD_ID`、`robot_init()`、`robot_diag_report()`；固件号需随发布版本人工更新 |
| [mission.c](App/mission.c) / [mission.h](App/mission.h) | 正式整场顺序、QR 三目标校验、路线腿和启动闸门 | `QR_*`、`R_*`、`ROUTE_FWD_V_MMS`、`ROUTE_STRAFE_V_MMS`、`CROSS_*`、`CAL_*_READY`；`mission_config_missing()` 会指出缺项 |
| [task_eod.c](App/task_eod.c) | 排爆：对球、预降、伸爪抓球、抬升、两次 180°、对桶放球、双轴按步数回起点 | `EOD_BALL_PRELOWER_STEPS`、`EOD_BALL_EXTEND_STEPS`、`EOD_LIFT_STEPS`、`EOD_LOWER_STEPS`（全待实测） |
| [task_anti.c](App/task_anti.c) | 反恐：对选定颜色靶、用节点停稳判据确认、激光射击 | `LASER_ON_MS`；停稳时序复用 `steps.c::step_prepare_leg()`，站位在 `steps.c` |
| [task_rescue.c](App/task_rescue.c) | 救援：对选定形状人质、预降、伸爪抓取、抬升并保持夹持；底盘回程由 `mission.c` 做 | `RESCUE_PRELOWER_STEPS`、`RESCUE_EXTEND_STEPS`、`RESCUE_LIFT_STEPS`（全待实测）；不放下、不做排爆式复位 |
| [robot_tasks.h](App/robot_tasks.h) | 三个任务的接口与返回码 | 只改接口，不放行程数值；**不要改名为 `task.h`**，会与 FreeRTOS 头文件撞名 |
| [steps.c](App/steps.c) / [steps.h](App/steps.h) | 共用步骤：视觉扫/对位、节点停稳、直走/横移/旋转、爪与两轴动作、激光；正式抓放前停稳、阶段回传 | `s_stand[4]`、`X_ALIGN_*`、`X_DEPTH_*`、`SWEEP_*`、`s_nav_w_kp_deg`、`s_orth_kp`、`ROT_*`、`ARM_STEP_INTERVAL_MS`、`AXIS1_*_DIR`、`step_arm_prepare/run/release()`；正式 180°已修连续航向方向歧义，但仍未集成模式22的速度参数或完成实车验收 |
| [auto_steps.c](App/auto_steps.c) / [auto_steps.h](App/auto_steps.h) | 越障：定速、锁向、pitch 峰峰值两阶段判定 | `X_RISE_DEG`、`X_FLAT_DEG`、`X_WIN_N`；越障速度/物理防跑飞时长在 `mission.c` |
| [motion.c](App/motion.c) / [motion.h](App/motion.h) | 麦轮运动学、前进/横移里程、位姿估计、启停加减速剖面 | `M_WHEEL_R_MM`、`M_A_HALF_MM`、`s_profile` (`acc/dec`)；`M_GEAR_RATIO` **保持 1.0**，不可再乘 30 |
| [control.c](App/control.c) / [control.h](App/control.h) | 四轮编码器测速和每轮速度闭环、刹车、开环单轮驱动 | `CTRL_ENCODER_CPR` 在 `.h`；`s_kp/s_ki/s_lp_alpha/s_dead_min` 在 `.c`，可先蓝牙 RAM 临时调 |
| [board_pins.c](App/board_pins.c) / [board_pins.h](App/board_pins.h) | 电机/编码器索引和极性、PWM/方向脚、STBY、激光、蓝牙 TX | `s_motor_inv`、`s_encoder_inv` 和 pin/TIM 数组；仅在单轮正向及编码器符号复核后改 |
| [arm.c](App/arm.c) / [arm.h](App/arm.h) | 两路步进 STEP/DIR 脉冲、爪舵机 PWM | `CLAW_OPEN_US`、`CLAW_CLOSE_US`、STEP/DIR pin 数组；当前机械挡块**没有**回零电信号 |
| [imu.c](App/imu.c) / [imu.h](App/imu.h) | IMU 串口帧解析、连续 yaw/pitch/roll、链路有效性、分段航向软件零点 | `IMU_LINK_TIMEOUT_MS`、协议字段；方向/零点须结合实车数据查，不靠改常数猜 |
| [proto.c](App/proto.c) / [proto.h](App/proto.h) | 当前视觉 AA55/CRC16 二进制接收、QR/OBJ 类别转换；保留旧 ASCII 解析用于回归，二进制模式尚无切场景命令 | `proto_set_binary_mode`、`CLS_*`、`LAB_*`、`ProtoFrame`；按 `VISION_INTEGRATION.md` 核对映射与目标工作点 |
| [test.c](App/test.c) / [test.h](App/test.h) | 蓝牙台架模式 1–29、`g` 控制、数据回传、RAM 调参；**不是**正式整场路线 | 执行器/命令解析在 `.c`；命令见下文，测试 `d/v` 不会写入 `mission.c` |
| [test_config.h](App/test_config.h) | 台架命令长度、模式上限、采样周期、默认速度/距离/补偿种子及上电安全开关集中入口 | `BENCH_AUTO=0` 必须保持；只改变台架默认，不会自动改变正式路线；右转 90°仍复用 `turn_profile.h` |
| [turn_profile.h](App/turn_profile.h) | 已做过落地测试的右转 90° 参数组，供测试模式20与正式 90°分支共用 | `TURN90_*`；模式22 的 180°只是候选，正式通用 180°参数在 `steps.c` |

`Src/main.c`/`Src/freertos.c` 是启动和任务周期，`Src/gpio.c`/`Src/tim.c`/`Src/usart.c` 与根目录 `jiejie.ioc` 是 CubeMX 外设配置；`Drivers/`、`Middlewares/` 是库。改引脚、定时器、串口时要同时核对 `.ioc`、生成代码和 `App/board_pins.c`/`arm.c`，不要只改其中一个。

## 要调什么，去哪里改

| 调试内容 | 台架/RAM 临时入口 | 最终归属（填实测值后全量重编） |
| --- | --- | --- |
| 单轮方向、编码器正负、轮位 | `test.c` 模式7–10、13，蓝牙 `diag` | `board_pins.c` 的电机/编码器映射与极性；`control.h` 的 `CTRL_ENCODER_CPR` |
| 轮速环 | 蓝牙 `kp/ki/lp/dead` → `param`，断电丢失 | `control.c` 的四个默认种子参数 |
| 底盘直走/横移距离、速度 | 模式15–18 的 `v`/`d`；`ykp/okp/acc/dec` 为 RAM-only；`lff/rff/fff` **只在指定模式和速度生效** | 轮径/运动学在 `motion.c`，航向/正交修正在 `steps.c`，正式路线段长/速度在 `mission.c`；测试 `d` 不会自动写回 |
| 起停加减速 | 蓝牙 `acc/dec` → `param`，仅 RAM | `motion.c` 的 `s_profile`；正式任务要求非零且实测确认 |
| 90°/180°车身自转 | 模式20/22 日志与实体角度 | 90°看 `turn_profile.h`；正式通用 180°看 `steps.c` 的 `ROT_*`；不能直接把模式22完成当正式 180°验收 |
| QR/球/靶/人质/桶视觉 | 核对手机/视觉串口原始帧及 `diag`；视觉识别算法由队友负责 | `proto.h` 类别/标签，`mission.c` QR 三元组，`steps.c` 的 `s_stand[4]`、对位与扫描参数 |
| 越障 | 先采 IMU pitch 与实车通过数据 | `auto_steps.c` 的 `X_RISE_DEG/X_FLAT_DEG`；`mission.c` 的 `CROSS_V_MMS/CROSS_TMO_MS` |
| 爪开合与两步进方向 | 蓝牙 `su<脉宽>`、`co/cc`；24/25 有限步不回，26/27 有限步等待 2 秒反走，步进方向/步数用 `nl5`（DIR0）或 `nr5`（DIR1）；28 舵机等待 2 秒回前一脉宽，29 舵机保持 | `arm.c` 爪脉宽与轴引脚、`steps.c` 方向/步间隔；`l/r` 只是测试命令别名，不代表已确认机械方向；勿用机械挡块撞停找零 |
| 球抓放 | 装车后逐段量下降、伸出、抬升、降桶和回程 | `task_eod.c` 的四个 `EOD_*` 步数；回程按已走步数反向，不是传感器绝对回零 |
| 人质抓取 | 装车后逐段量下降、伸出、抬升 | `task_rescue.c` 的三个 `RESCUE_*` 步数；抓后保持夹持，不放人/复位 |
| 激光 | 先验证编码器停稳、实物余晃与实际照射 | `task_anti.c` 的亮灯时间，停稳判据及目标站位在 `steps.c` |

当前已知的调试命令可在蓝牙发 `?` 让固件自己列出；发 `param` 查 RAM 参数，发 `diag` 查固件号与链路。**只有在空闲/BOOT 时**才可选号与调参；`g` 的含义随模式变化，上电不会自动转轮（`BENCH_AUTO=0`）。不要把示例命令当作允许未经架空/落地风险检查就启动机械。

## 配置状态与安全边界

- 正式整场入口是 `mission.c::mission_start()`；`mission_config_missing()` 会拒绝仍为 0 的路线、扫描、球/人质行程等关键数值，还要四个 `CAL_*_READY` 人工确认。不要为了“能跑”直接把闸门改成 1。
- 目前球/人质关键行程仍为 0。2026-09-30 正式抓球、对桶放球、救援抓人质的视觉对位后，已接入现有 `step_prepare_leg()`：先刹车、观察编码器静止 250 ms、IMU 本段软件清零、再等 750 ms，失败或中止就不发机械动作。各抓放动作回传 `REC type=ARM` 阶段和 `ARM_PULSE/ARM_CLAW` 命令记录；`COMMAND_SENT`、`ALL_PULSES_SENT` **仅表示控制命令/脉冲发出**，`physical_unverified=1` 明确表示没有位置传感器证明到位。中途停止后的机械位置不确定，须人工重新确认起点。
- 抓放阶段记录版 Keil 全量重编日志 `MDK-ARM/rebuild_arm_stage_trace_2026-09-30.txt` 为 0 Error / 0 Warning；主机端 `tests/arm_task_flow_test.c` 检查排爆、救援顺序及两个中止点，共 4 例通过。但当前配置闸门仍关闭，链接 map 将正式任务函数裁掉；这只是源码/编译检查，**不是已烧录或实车抓放验收**。填入实测行程后须重新全量重编，确认 map 中正式任务已链接，再做实体测试。
- 2026-09-30 软件复核修复了 `g` 后立刻 `a` 的启动竞态：原 `MissionTask` 醒来会再次调用 `run_reset()`，可能擦掉已收到的急停。现在只在初始化和**接受启动请求之前**清旧标志；启动锁存后立刻检查中止，不再清零。整场 `a` 先置中止并立即下发底盘刹车，蓝牙回 `OK ABORT_REQUEST brake_commanded; physical_stop_unverified`，不谎报物理已停稳。越障循环也改为**先检查急停/防跑飞时限，再发本拍速度**。`tests/obstacle_abort_test.c` 主机 5 例通过；尚未烧录。上车首次整场联调需专门做一次安全架空 `g`→立即 `a` 验证。当前所有标定闸门仍保持关闭，勿为此短测擅自放行整场。
- 反恐视觉对靶后也改为复用 `step_prepare_leg()`，不再只盲等 1 秒就发射。在工作工程根目录运行 `tests/run_host_tests.ps1`，可重复检查排爆/救援 4 例、反恐 2 例、越障中止/时限 5 例、实际 `mission.c` 启动瞬间急停 1 例、QR 三位映射 27 个合法/7 个非法组合、视觉 ASCII 解析 9 行和源码顺序 4 项。启动急停主机测试只注入“启动请求已接受”状态，**正式参数闸门原样关闭**；这些仍非实车验收。
- 2026-09-30 解析复核修复 `proto.c`：数字字段格式错误的 QR/OBJ/ERR 行原会以 `PF_NONE` 被计作已接受，现在默认 `PF_UNKNOWN` 并计入拒收，不传给业务回调。此修复初版固件号 `20260930-PROTO-REJECT-FIX`；Keil 全量日志 `MDK-ARM/rebuild_proto_reject_fix_2026-09-30.txt` 为 0 Error / 0 Warning，未烧录。视觉队友仍须给实际 `QR,d1,d2,d3` 与四类 `OBJ` 样帧，确认标签、置信度、相机中心/尺寸、帧率与场景切换后首帧时序；协议测试不能替代联调。
- 2026-09-30 框架整理把 `test.c` 内台架常量原值移入 `test_config.h`，不动已验证的速度/控制律；同步修正 `steps.h` 曾误称 `step_sweep` 可扫 QR 的过时说明。有限 `to` 的视觉对位以前可能在等目标帧时无限卡住，现匹配帧等待会按时限退出，纵向移动也只使用剩余时限；正式任务当前传 `to=0`，行为不变。新增 `tests/align_timeout_test.c` 主机 1 例通过；全套主机回归通过，Keil 全量日志 `MDK-ARM/rebuild_framework_audit_2026-09-30.txt` 为 0 Error / 0 Warning。HEX SHA-256 仍为 `DB040A88C642C19B39564851EAC190CD94D2EADCA83013CC4071C06DABDA9AA8`（当前配置闸门使这段正式视觉流程被链接器裁掉），未烧录或实车验收。
- 正式普通路线原来一份 `ROUTE_V_MMS` 混用直走与横移，现分成 `ROUTE_FWD_V_MMS`、`ROUTE_STRAFE_V_MMS`，均继续置 0 并纳入启动闸门。你已有的“直走 `v200`、平移 `v300`”是分开设置的依据，但不等于后半程所有路段已验收，故本次不擅自填正式速度。该修改初版固件号 `20260930-ROUTE-SPEED-SPLIT`；Keil 全量日志 `MDK-ARM/rebuild_route_speed_split_2026-09-30.txt` 为 0 Error / 0 Warning，尚未烧录。主机测试另查 3 个后半程直行调用、6 个横移调用都引用对应速度及双闸门。
- 正式通用 `step_rotate_deg(±180)` 原用 0–360° yaw 的最短角差，目标刚好 180°时容易因起转前约 1°噪声选择反方向。现改用 IMU 连续航向减起始航向，保持“正 180 就正向、负 180 就反向”；主机假 IMU 的 3 个首拍方向测试通过。**这里只修角度口径，未套用模式22的速度/超时参数，也未实车带球验证。**当前最新固件号 `20260930-TURN180-CONTINUOUS`；Keil 全量日志 `MDK-ARM/rebuild_turn180_continuous_2026-09-30.txt` 为 0 Error / 0 Warning。正式任务参数仍为 0，map 将 `step_rotate_deg`/三个任务函数裁掉，当前 HEX 不包含这段正式控制代码；填实测参数放行后必须重编确认已链接。
- 台架模式 24–27 限单次 50 步，发 `nl1..nl50` 指 DIR0、`nr1..nr50` 指 DIR1，旧 `n-5`/`n5` 分别等价 `nl5`/`nr5`。方向、轴号、实际位移及挡块余量都须先目视确认；26/27 完整走完后等待 2 秒，再反走同样的**命令步数**。途中按 `g/a/0` 立即停止后续脉冲并取消自动回程；无电气零点，暂停、卡住或断电后从人工标记重新确认起点。
- 模式 28/29 先用 `u1000..u1800` 设置目标，再 `g` 发舵机 PWM 指令。28 等 2 秒回到动作前**指令脉宽**；29 保持。等待中 `g/a/0` 取消回程但保持当前 PWM，不是切电或保证舵机机械立即停住。先不装爪/连杆测，小范围渐进；`su` 为原有即时手动指令。
- 改代码后全量 Rebuild，核对日志、固件识别号、HEX 哈希；上车仍要分别验证方向、行程、碰撞包络，编译通过不代表硬件正确。
