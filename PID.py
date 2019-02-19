#!/usr/bin/python
#
# This file is part of IvPID.
# Copyright (C) 2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
#
# IvPID is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IvPID is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# title           :PID.py
# description     :python pid controller
# author          :Caner Durmusoglu
# date            :20151218
# version         :0.1
# notes           :
# python_version  :2.7

"""
＃！/ usr / bin / python
＃
＃このファイルはIvPIDの一部です。
＃Copyright（C）2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
＃
＃IvPIDはフリーソフトウェアです：あなたはそれを再配布したり変更することができます
＃によって公開されているGNU一般公衆利用許諾契約書の条項の下で。
＃フリーソフトウェア財団、ライセンスのバージョン3、または
＃（あなたの選択で）それ以降のバージョン。
＃
＃IvPIDが役に立つことを願って配布されています。
＃ただし、いかなる保証もありません。 の黙示的な保証もありません。
＃商品性または特定の目的への適合性。 を参照してください
＃詳細についてはGNU一般公衆利用許諾契約書。
＃
＃あなたはGNU General Public Licenseのコピーを受け取っているはずです。
＃このプログラムと一緒に。 そうでない場合は、<http://www.gnu.org/licenses />を参照してください。

＃title：PID.py
＃説明：python pidコントローラー
＃著者：Caner Durmusoglu
＃date：20151218
＃バージョン：0.1
＃ ノート           ：
＃python_version：2.7
"""
# ==============================================================================

"""
Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller in the Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller

vmech PID Controllerは、Pythonプログラミング言語のProportional-Integral-Derivative（PID）コントローラーの単純な実装です。
PIDコントローラーに関する詳細情報：http://en.wikipedia.org/wiki/PID_controller
"""
import time

class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):  #値の設定

        self.Kp = P  #比例制御の比例定数
        self.Ki = I  #積分制御の比例定数
        self.Kd = D  #微分制御の比例定数

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        """
        sample_time　測定しているものの更新時間。データが更新されてないのに余計な計算をするのを防ぐための値
        current_time 現在の時間
        last_time　　　一つ前の時間
        
        """
        self.clear()

    def clear(self):  #初期値設定
        """Clears PID computations and coefficients  PID計算と係数をクリアします"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard  ワインドアップガード 終わるのを防ぐ？
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0
        
        

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
           
           与えられた参照フィードバックのPID値を計算します

         ..数学::
             u（t）= K_p e（t）+ K_i \ int_ {0} ^ {t} e（t）dt + K_d {de} / {dt}
　　　　　　　　　　　　　　　　　　　注    \ int_ {0} ^ {t} e（t）dt　はe(t)を0からtまで積分したもの
以下のプログラムで
PTerm =K_p e（t）
ITerm =\ int_ {0} ^ {t} e（t）dt
DTerm ={de} / {dt}
と定義している

				

         .. figure :: images / pid_1.png
            ：整列：中央

            Kp = 1.2、Ki = 1、Kd = 0.001のテストPID（test_pid.py）

        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        """
        error = self.SetPoint - feedback_value        目標値と現在の値との差

        self.current_time = time.time()　　現在の時間
        delta_time = self.current_time - self.last_time　　　デルタ t
        delta_error = error - self.last_error　　　　　　　　　デルタ 目標値と現在の値との差
        """

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error    #K_p e（t）
            self.ITerm += error * delta_time  #\ int_ {0} ^ {t} e（t）dt つまり e(t) * dtを1ループごとに足す

            if (self.ITerm < -self.windup_guard):  #ITermの絶対値が大きくなりすぎた場合はwindup_guardまでおさえる　積分は誤差が出やすい
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time  #{de} / {dt}

            # Remember last time and last error for next calculation 次の計算のために前回と前回のエラーを記憶する
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)  #変化すべき量を計算。この関数の初めの方にあるコメントの数式と同じもの
            

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain 
        比例ゲインを設定して、PIDが現在のエラーにどの程度積極的に反応するかを決定します。"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain 
        Integral Gainを設定して、PIDが現在のエラーにどれだけ積極的に反応するかを決定します。"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain
        微分ゲインを設定して、PIDが現在のエラーにどれだけ積極的に反応するかを決定します。
        """
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        
        積分ワインドアップ。積分ワインドアップまたはリセットワインドアップとも呼ばれます。
         PIDフィードバックコントローラ内の状況を指します。
         設定値に大きな変化がある（正の変化を言う）
         そして積分項はかなりの誤差を累積する
         上昇（ワインドアップ）の間、このようにオーバーシュートと継続
         この累積誤差がほどけるにつれて増加する
         （反対方向の誤差によって相殺される）。
         具体的な問題は、過剰なオーバーシュートです。
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        
        定期的に更新されるべきPID。
         予め決められたサンプリング時間に基づいて、ＰＩＤは、それがすぐに計算するべきかそれとも戻るべきかを決定する。
        """
        self.sample_time = sample_time
