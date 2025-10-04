import sympy as sp
from forward_kinematics_dh_class import ForwardKinematicsDH

# ================================
# MATRIZ INICIAL (IDENTIDAD)
# ================================
matriz_inicial = sp.eye(4)
print("\n--- Matriz inicial (Identidad) ---")
sp.pprint(matriz_inicial, use_unicode=True)

# ================================
# ROBOT RR (Planar)
# ================================
print("\n=== Robot Planar RR ===")
q1, q2 = sp.symbols('q1 q2')
l1, l2 = sp.symbols('l1 l2')

dh_params_rr = [
    [q1, 0, l1, 0],
    [q2, 0, l2, 0],
]

H_rr = ForwardKinematicsDH.symbolic(dh_params_rr)
H_rr = matriz_inicial * H_rr

pos_rr = sp.Matrix([H_rr[0, 3], H_rr[1, 3]])
q_rr = sp.Matrix([q1, q2])
J_rr = pos_rr.jacobian(q_rr)

print("\nMatriz Homogénea H_rr:")
sp.pprint(H_rr, use_unicode=True)
print("\nPosición pos_rr:")
sp.pprint(pos_rr, use_unicode=True)
print("\nJacobian J_rr:")
sp.pprint(J_rr, use_unicode=True)

# ================================
# ROBOT RRR (Antropomórfico)
# ================================
print("\n=== Robot Antropomórfico RRR ===")
q1, q2, q3 = sp.symbols('q1 q2 q3')
l1, l2, l3 = sp.symbols('l1 l2 l3')

dh_params_rrr = [
    [q1, 0, l1, 0],
    [q2, 0, l2, 0],
    [q3, 0, l3, 0],
]

H_rrr = ForwardKinematicsDH.symbolic(dh_params_rrr)
H_rrr = matriz_inicial * H_rrr

pos_rrr = H_rrr[0:3, 3]
q_rrr = sp.Matrix([q1, q2, q3])
J_rrr = pos_rrr.jacobian(q_rrr)

print("\nMatriz Homogénea H_rrr:")
sp.pprint(H_rrr, use_unicode=True)
print("\nPosición pos_rrr:")
sp.pprint(pos_rrr, use_unicode=True)
print("\nJacobian J_rrr:")
sp.pprint(J_rrr, use_unicode=True)

# ================================
# ROBOT RRP (SCARA)
# ================================
print("\n=== Robot SCARA RRP ===")
q1, q2, q3 = sp.symbols('q1 q2 q3')
l1, l2 = sp.symbols('l1 l2')

dh_params_rrp = [
    [q1, 0, l1, 0],
    [q2, 0, l2, 0],
    [0, -q3, 0, 0],
]

H_rrp = ForwardKinematicsDH.symbolic(dh_params_rrp)
H_rrp = matriz_inicial * H_rrp

pos_rrp = H_rrp[0:3, 3]
q_rrp = sp.Matrix([q1, q2, q3])
J_rrp = pos_rrp.jacobian(q_rrp)

print("\nMatriz Homogénea H_rrp:")
sp.pprint(H_rrp, use_unicode=True)
print("\nPosición pos_rrp:")
sp.pprint(pos_rrp, use_unicode=True)
print("\nJacobian J_rrp:")
sp.pprint(J_rrp, use_unicode=True)