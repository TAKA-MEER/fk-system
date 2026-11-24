# auto_runner -> 画面。正本は docs/plan/detailed/DetailedDesign-names.md §4.1。
std_msgs/Header header
string phase
string panel_id          # いま向かっている・作業中の盤
string[] queue           # 残りの順
string side              # いまの訪問の側。front / back。訪問が無ければ空
string[] queue_side      # queue と同じ並びの側
string halt_reason
string gate_reason       # 関門で止めている理由。無ければ空
bool lift_at_bottom
bool lift_readable
bool arm_stowed
bool mechanism_attached
bool reentry_required
string th_mode
string th_state
string th_reject_reason  # th が最後に指示を拒否した理由。th の理由キーのまま。無ければ空
bool mechanism_bypass    # 本システムの設定（OPT: mechanism_bypass。DetailedDesign-auto.md §2.3）
