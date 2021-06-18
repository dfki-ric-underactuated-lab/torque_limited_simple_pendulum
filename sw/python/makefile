test: test_model test_trajopt_iLQR test_trajopt_dircol test_con_energy test_con_lqr test_con_ilqr

test_model:
	python -m unittest model/unittest.py

test_trajopt_iLQR:
	python -m unittest trajectory_optimization/iLQR/unittest.py

test_trajopt_dircol:
	python -m unittest trajectory_optimization/direct_collocation/unittest.py

test_con_energy:
	python -m unittest controllers/energy_shaping/unittest.py

test_con_lqr:
	python -m unittest controllers/LQR/unittest.py

test_con_ilqr:
	python -m unittest controllers/iLQR/unittest.py
