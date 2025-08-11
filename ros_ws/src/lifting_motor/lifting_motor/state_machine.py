from enum import Enum

# このコードには状態を定義するState, また状態遷移を管理するStateMachineクラスが含まれる
# 状態遷移に関する処理以外は絶対に記述しない
# 状態遷移時に副作用をもたせたい場合は後述のjust_entered_state等を用いてliftin_motor.pyに記述すること


class State(Enum):
    INIT = 0
    STOPPED = 1
    TO_MAX = 2
    RETURN_TO_MIN = 3

class StateMachine:
    def __init__(self):
        # 初期状態の設定(INIT)
        self.state = State.INIT
        self.previous_state = State.INIT  # 前回の状態を保持

    def update_state(self, inputs):
        # 前回の状態を保存
        self.previous_state = self.state
        # 安全ボタンが押された場合は常にINITに戻る（最高優先度）
        if inputs.get('safety_reset_button', False): # もしこれがTrueなら実行される
            self.state = State.INIT
            return  # 他の条件をチェックしない
        
        # 緊急停止の場合は常にSTOPPEDに戻る（2番目の優先度）
        elif inputs.get('emergency_stop', False):
            self.state = State.STOPPED
            return  # 他の条件をチェックしない
        
        # 初期化完了でSTOPPED状態へ
        elif self.state == State.INIT and inputs.get('is_system_ready', False):
            self.state = State.STOPPED

        # 間違えてSTOPPEDのループに入ってしまったとき用に初期化状態に戻す
        elif self.state == State.STOPPED and inputs.get('is_system_ready', False):
            self.state = State.INIT
            
        # 射出シーケンス開始の条件
        elif self.state == State.STOPPED and \
                not inputs.get('is_throwing_motor_on', False) and \
                inputs.get('is_elevation_minlim_on', False) and \
                inputs.get('is_ejection_on', False):
            self.state = State.TO_MAX
            
        # 最大位置に到達したら戻り開始
        elif self.state == State.TO_MAX and inputs.get('is_ejection_maxlim_on', False):
            self.state = State.RETURN_TO_MIN
            
        # 最小位置に戻ったらSTOPPED状態へ
        elif self.state == State.RETURN_TO_MIN and inputs.get('is_ejection_minlim_on', False):
            self.state = State.STOPPED
    
    def get_current_state(self):
        """現在の状態を取得"""
        return self.state
    
    def get_state_name(self):
        """現在の状態名を文字列で取得"""
        return self.state.name
    
    def reset_to_init(self):
        """強制的にINIT状態にリセット"""
        self.state = State.INIT
    
    def is_safe_to_operate(self):
        """動作可能な状態かチェック"""
        return self.state in [State.STOPPED, State.TO_MAX, State.RETURN_TO_MIN]
    
    def is_in_init_state(self):
        """INIT状態かどうかチェック（安全確認用）"""
        return self.state == State.INIT
    
    def has_state_changed(self):
        """状態が変化したかチェック"""
        return self.state != self.previous_state
    
    # 副作用的処理を追加する際に用いる
    def just_entered_state(self, target_state):
        """指定した状態に今回の更新で入ったかチェック"""
        return self.state == target_state and self.previous_state != target_state
    
    def just_exited_state(self, target_state):
        """指定した状態から今回の更新で出たかチェック"""
        return self.previous_state == target_state and self.state != target_state
    
    def get_previous_state(self):
        """前回の状態を取得"""
        return self.previous_state
    
    def get_state_transition_info(self):
        """現在の状態と可能な遷移を返す（デバッグ用）"""
        transitions = {
            State.INIT: "INIT → システム準備完了でSTOPPEDへ",
            State.STOPPED: "STOPPED → 射出条件が揃うとTO_MAXへ",
            State.TO_MAX: "TO_MAX → 最大リミット到達でRETURN_TO_MINへ",
            State.RETURN_TO_MIN: "RETURN_TO_MIN → 最小リミット到達でSTOPPEDへ"
        }
        return f"現在: {self.get_state_name()}, 次: {transitions.get(self.state, '不明')}"