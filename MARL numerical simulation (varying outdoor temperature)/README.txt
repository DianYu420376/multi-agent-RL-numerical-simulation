Major functions implementing the MARL algorithm are:
	estimate_gradient (Use single point zero-th order method)
	estimate_gradient_coordinate (Use coordination difference for zero-th order method, not used in testing)
	estimate_cost (estimate the cost by running trajectory and doing consensus)
	sample_uniform_sphere (generate sample from uniform sphere distribution, currently do not use consensus)
	

Function and file for defining the Building model:
	Building-4-room-changing-outside-temperature.m, Data file, specifying the A, B, W matrix and indoor gain and outdoor temperature distribution for a toy model of 4-room building
	Building, function that realizes the building dynamic
	temperature_data.m, Data file, specifying outdoor temperature sequence for testing and visualization


File for testing the result:
	test.m --> RUN THIS FILE TO GET THE FIGURES IN THE PAPER.

Function for visualization of the result:
	generate_traj_real_time.m
	generate_traj_real_time_comparing.m (Compare the controller with a static state feed back controller, K_comparison)