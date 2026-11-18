"""MPC-регулятор СДП (линейный, offset-free).

Модель в NED-кадре (матрица поворота замораживается на горизонте):
    eta_dot = v
    v_dot   = -Mn_inv @ Dn @ v + Mn_inv @ (tau + d)
где Mn_inv = R Minv Rᵀ, Dn = R D Rᵀ, d — оценка возмущения (ветер + bias EKF).

Формулировка в приращениях Δu (гладкость управления), горизонт N шагов
по dt сек. QP с бокс-ограничениями на Δu решается FISTA-проекцией
(только numpy). Ограничения по абсолютной тяге — консервативный клиппинг
(точный учёт возможностей ДРУ — в аллокаторе).
"""
import numpy as np

from .vessel import Vessel3DOF


class MPCController:
    def __init__(self, vessel: Vessel3DOF, dt=0.5, N=20,
                 q_pos=(1.0, 1.0, 60.0), q_vel=(0.0, 0.0, 3.0e5),
                 r_du=1e-12,
                 du_max=(5.0e4, 5.0e4, 5.0e5),
                 tau_max=(4.0e5, 3.5e5, 1.2e7),
                 n_sub=20, fista_iter=80):
        self.v = vessel
        self.dt = dt
        self.N = N
        self.q = np.asarray(q_pos, float)
        self.qv = np.asarray(q_vel, float)
        self.r_du = r_du
        self.du_max = np.asarray(du_max, float)
        self.tau_max = np.asarray(tau_max, float)
        self.n_sub = n_sub
        self.fista_iter = fista_iter
        self.u_prev = np.zeros(3)      # последний tau в NED

    # --- Дискретизация ------------------------------------------------------
    def _discretize(self, psi):
        R = Vessel3DOF.R(psi)
        Mn_inv = R @ self.v.Minv @ R.T
        Dn = R @ self.v.D @ R.T
        Z = np.zeros((3, 3))
        I = np.eye(3)
        A_c = np.block([[Z, I],
                        [Z, -Mn_inv @ Dn]])
        B_c = np.vstack([Z, Mn_inv])
        s = self.dt / self.n_sub
        P = np.eye(6) + A_c * s
        A_d = np.linalg.matrix_power(P, self.n_sub)
        # B_d = sum_{i=0}^{n_sub-1} P^i B_c s
        B_d = np.zeros((6, 3))
        Pk = np.eye(6)
        for _ in range(self.n_sub):
            B_d += Pk @ B_c * s
            Pk = Pk @ P
        return A_d, B_d

    # --- QP: FISTA с проекцией на бокс --------------------------------------
    @staticmethod
    def _fista(H, f, lo, hi, iters):
        L = float(np.linalg.eigvalsh(H)[-1])
        z = np.clip(-np.linalg.solve(H, f), lo, hi)   # тёплый старт
        y = z.copy()
        t = 1.0
        for _ in range(iters):
            z_new = np.clip(y - (H @ y + f) / L, lo, hi)
            t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t * t))
            y = z_new + ((t - 1.0) / t_new) * (z_new - z)
            z, t = z_new, t_new
        return z

    # --- Основной вызов ------------------------------------------------------
    def compute(self, eta_d, eta_hat, nu_hat, d_hat):
        """tau (body-кадр) по оценкам наблюдателя и возмущения d_hat (NED)."""
        psi = eta_hat[2]
        R = Vessel3DOF.R(psi)
        v_ned = R @ np.asarray(nu_hat, float)
        A_d, B_d = self._discretize(psi)

        N = self.N
        # Расширенная модель [eta, v, u_prev] с входом Δu
        A_t = np.block([[A_d, B_d],
                        [np.zeros((3, 6)), np.eye(3)]])
        B_t = np.vstack([B_d, np.eye(3)])
        C = np.hstack([np.eye(3), np.zeros((3, 6))])
        E_t = np.vstack([B_d, np.zeros((3, 3))])

        X0 = np.concatenate([np.asarray(eta_hat, float), v_ned, self.u_prev])
        d = np.asarray(d_hat, float)

        # Конденсация горизонта (выход — весь вектор состояния [eta, v]):
        # X_i = A^i X0 + sum_{k<=i} A^{i-k} B du_k + sum_{k<=i} A^k E d
        AB_p = [B_t]                       # AB_p[p] = A^p B
        AE_p = [E_t]                       # AE_p[p] = A^p E
        for _ in range(N - 1):
            AB_p.append(A_t @ AB_p[-1])
            AE_p.append(A_t @ AE_p[-1])
        ns = 6                             # размер выхода [eta, v]
        Phi = np.zeros((ns * N, 9))
        G = np.zeros((ns * N, 3 * N))
        Gam = np.zeros((ns * N, 3))
        Co = np.hstack([np.eye(6), np.zeros((6, 3))])
        Apow = np.eye(9)
        AE_sum = np.zeros((9, 3))
        for i in range(N):
            Apow = Apow @ A_t              # A^{i+1}
            Phi[ns * i:ns * i + ns] = Co @ Apow
            for k in range(i + 1):
                G[ns * i:ns * i + ns, 3 * k:3 * k + 3] = Co @ AB_p[i - k]
            AE_sum += AE_p[i]
            Gam[ns * i:ns * i + ns] = Co @ AE_sum

        Qstep = np.diag(np.concatenate([self.q, self.qv]))
        Qb = np.kron(np.eye(N), Qstep)
        Rb = self.r_du * np.eye(3 * N)
        yref = np.tile(np.concatenate([np.asarray(eta_d, float),
                                       np.zeros(3)]), N)

        H = 2.0 * (G.T @ Qb @ G + Rb)
        f = 2.0 * G.T @ Qb @ (Phi @ X0 + Gam @ d - yref)

        lo = -self.du_max.repeat(N)
        hi = self.du_max.repeat(N)
        du = self._fista(H, f, lo, hi, self.fista_iter)

        tau_ned = self.u_prev + du[:3]
        tau_ned = np.clip(tau_ned, -self.tau_max, self.tau_max)
        self.u_prev = tau_ned
        return R.T @ tau_ned

    def reset(self):
        self.u_prev[:] = 0.0
