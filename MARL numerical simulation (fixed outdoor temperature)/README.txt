Major functions implementing the MARL algorithm are:
	estimate_gradient (Use single point zero-th order method)
	estimate_gradient_coordinate (Use coordination difference for zero-th order method, not used in testing)
	estimate_cost (estimate the cost by running trajectory and doing consensus)
	sample_uniform_sphere (generate sample from uniform sphere distribution, currently do not use consensus)
	test_calculate_cost (Calculate the real cost J(K) corresponding to a controller K using 'dlyap' function)
	test_calculate_gradient (Calculate the real gradient \nabla J(K), also using 'dlyap' function)

Function and file for defining the Building model:
	Building-4-room.m, Data file, specifying the A, B, W matrix for a toy model of 4-room building
	Building, function that realizes the building dynamic


File for testing the result:
	test_pretty_plots.m --> RUN THIS FILE TO GET THE FIGURES IN THE PAPER.

Function for visualization of the result:
	generate_traj.m