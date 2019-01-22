# -*- coding: utf-8 -*-
"""
    Copyright (c) 2015 Jonas Böer, jonas.boeer@student.kit.edu
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    著作権（c）2015 JonasBöer、jonas.boeer@student.kit.edu
     このプログラムはフリーソフトウェアです：あなたはそれを再配布することができますおよび/または変更することができます
     によって公開されているGNU劣等一般公衆利用許諾契約書の条項に基づきます。
     Free Software Foundation、ライセンスのバージョン3、または
     （あなたの選択で）それ以降のバージョン。
     このプログラムは、役に立つことを願って配布されています。
     しかし、いかなる保証もありません。 黙示の保証もありません。
     商品性または特定の目的への適合性。 を参照してください
     詳細については、GNU劣等一般公衆利用許諾契約書。
     あなたはGNU劣等一般公衆利用許諾契約書のコピーを受け取ったはずです
     このプログラムと一緒に。 そうでない場合は、<http://www.gnu.org/licenses />を参照してください。
"""

import warnings
import numpy as np
from numpy.linalg import norm
from .quaternion import Quaternion


"""
import warnings                         エラー用
import numpy as np　　　　　　　　　　　　　　array(配列)関数を使う
from numpy.linalg import norm　　　　　　　ノルム、絶対値を計算
from .quaternion import Quaternion　　　　四元数←同サイトにあるものを使えば良い？

"""


class MadgwickAHRS:
    samplePeriod = 1/256
    quaternion = Quaternion(1, 0, 0, 0)
    beta = 1

    def __init__(self, sampleperiod=None, quaternion=None, beta=None):
        """
        Initialize the class with the given parameters.
        :param sampleperiod: The sample period
        :param quaternion: Initial quaternion
        :param beta: Algorithm gain beta
        :return:	
        
        与えられたパラメータでクラスを初期化します。
         ：param sampleperiod：サンプル期間
         ：paramクォータニオン：初期クォータニオン
         ：param beta：アルゴリズムゲインベータ
         ：return：
        """
        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta

    def update(self, gyroscope, accelerometer, magnetometer):　　　　　　　　　　#９軸の場合の関数（使わない）
        """
        Perform one update step with data from a AHRS sensor array
        :param gyroscope: A three-element array containing the gyroscope data in radians per second.
        :param accelerometer: A three-element array containing the accelerometer data. Can be any unit since a normalized value is used.
        :param magnetometer: A three-element array containing the magnetometer data. Can be any unit since a normalized value is used.
        :return:
        
        AHRSセンサーアレイからのデータを使用して1つの更新手順を実行する　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　AHRSセンサーアレイ＝ジャイロスコープ
         ：param gyroscope：ジャイロスコープのデータを1秒あたりのラジアンで含む3要素の配列。
         ：param accelerometer：加速度計データを含む3要素の配列。 正規化された値が使用されるので、どんな単位でも構いません。
         ：param magnetometer：磁力計データを含む3要素の配列。 正規化された値が使用されるので、どんな単位でも構いません。　　　　　　←磁気センサーは今回ないので修正の必要あり
         ：return：
        """
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()
		#magnetometer = np.array(magnetometer, dtype=float).flatten()

        # Normalise accelerometer measurement　加速度計の測定値を正規化する
        if norm(accelerometer) is 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Normalise magnetometer measurement　磁力計測定を正規化する
        """
        if norm(magnetometer) is 0:
            warnings.warn("magnetometer is zero")
            return
        magnetometer /= norm(magnetometer)
		
        h = q * (Quaternion(0, magnetometer[0], magnetometer[1], magnetometer[2]) * q.conj())
        b = np.array([0, norm(h[1:3]), 0, h[3]])
        """

        # Gradient descent algorithm corrective step　勾配降下アルゴリズム修正ステップ
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2],
            2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]) - magnetometer[0],
            2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]) - magnetometer[1],
            2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2) - magnetometer[2]
          
        ])
        j = np.array([
            [-2*q[2],                  2*q[3],                  -2*q[0],                  2*q[1]],
            [2*q[1],                   2*q[0],                  2*q[3],                   2*q[2]],
            [0,                        -4*q[1],                 -4*q[2],                  0]	,
            [-2*b[3]*q[2],             2*b[3]*q[3],             -4*b[1]*q[2]-2*b[3]*q[0], -4*b[1]*q[3]+2*b[3]*q[1]],
            [-2*b[1]*q[3]+2*b[3]*q[1], 2*b[1]*q[2]+2*b[3]*q[0], 2*b[1]*q[1]+2*b[3]*q[3],  -2*b[1]*q[0]+2*b[3]*q[2]],
            [2*b[1]*q[2],              2*b[1]*q[3]-4*b[3]*q[1], 2*b[1]*q[0]-4*b[3]*q[2],  2*b[1]*q[1]]
            
        ])
        step = j.T.dot(f)  #転置
        step /= norm(step)  # normalise step magnitude　ステップの大きさを正規化する

        # Compute rate of change of quaternion　四元数の変化率を計算する
        qdot = (q * Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion　四元数を生成するために積分する
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # normalise quaternion　四元数を正規化する

    def update_imu(self, gyroscope, accelerometer):　　　　　　　　　　　　　　　　　#６軸の場合の関数
        """
        Perform one update step with data from a IMU sensor array
        :param gyroscope: A three-element array containing the gyroscope data in radians per second.
        :param accelerometer: A three-element array containing the accelerometer data. Can be any unit since a normalized value is used.
        
        IMUセンサーアレイからのデータを使用して1つの更新手順を実行する　　　　　　　　　　　　　　　　IMUセンサーアレイ＝加速度計
         ：param gyroscope：ジャイロスコープのデータを1秒あたりのラジアンで含む3要素の配列。
         ：param accelerometer：加速度計データを含む3要素の配列。 正規化された値が使用されるので、どんな単位でも構いません。
        """
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        ac

        # Normalise accelerometer measurement　加速度計の測定値を正規化する
        if norm(accelerometer) is 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Gradient descent algorithm corrective step　勾配降下アルゴリズム修正ステップ
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2]
        ])
        j = np.array([
            [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
            [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
            [0, -4*q[1], -4*q[2], 0]
        ])
        step = j.T.dot(f)                     #j.T jの転置行列,　x.dot(f) xとｆの行列の積　つまりj.T.dot(f)はjとf
        step /= norm(step)  # normalise step magnitude　ステップの大きさを正規化する

        # Compute rate of change of quaternion　四元数の変化率を計算する
        qdot = (q * Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion　四元数を生成するために積分する
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # normalise quaternion　四元数を正規化する
