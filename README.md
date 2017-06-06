
# CarND-MPC-Control-Project
## 1.Introduction
    The objective of this project was to implement a MPC controller in c++ to make the car driving safely on track in the simulator provided from Udacity. The implementation of the project is in the `MPC.h`,'MPC.cpp','main.cpp', and the reflection  is in this README.MD ,and the output video is in the file `output_video.mp4` .

## 2. MPC Model Design
    A MPC controller can take car dynamics  and the enviroment factors into account and establish a control model to make the system control no more "black box". It can predict the car's a series of future timestep states so that MPC controller can be more forecasting and robust than PID controller.
	In this project, the MPC model is designed as follows:

	The MPC controller has two key steps,Evaluation part  and Solve part.
	
### 2.1 Evaluation

	In this part,we define our aim of cost function and our predictive model.

#### 2.1.1 Cost funtion

   I build a cost function to make the final cost least, which is the aim of our optimal control.
	the cost function takes into the following considerations:
	fg[0] = 0; //initialize cost function
	
	1) the closer to reference states,the better:

     for (int i = 0;   i < N;  i++) {
         fg[0] += hyperparam_cost_ref_cte   * CppAD::pow( vars[cte_start + i]   - ref_cte,  2) ;
         fg[0] += hyperparam_cost_ref_epsi  * CppAD::pow( vars[epsi_start + i]  - ref_epsi,  2) ;
         fg[0] += hyperparam_cost_ref_v     * CppAD::pow( vars[v_start + i ]     - ref_v,  2) ;
       }
	   
	2) the less using actuator, the better:

      for (int i = 0; i < N - 1; i++) {
        fg[0] += hyperparam_cost_ref_val_steering * CppAD::pow(vars[delta_start + i], 2);
        fg[0] += hyperparam_cost_ref_val_throttle * CppAD::pow(vars[a_start + i], 2);
      }
	  
	3) the smoothier of sequential actuation output, the better:

      for (int i = 0; i < N - 2; i++) {
        fg[0] += hyperparam_cost_ref_seq_steering * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
        fg[0] +=hyperparam_cost_ref_seq_throttle * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
      }
	
	here we introduce the hyperparameter  coefficients for each aim factor to trade between all the factors and gain the satisfied car running behavior.
  
	After lots of manual trying ,finally i set the  hyperparameters as follows:
  
	const double hyperparam_cost_ref_cte           =200;
	const double hyperparam_cost_ref_epsi          = 300;
	const double hyperparam_cost_ref_v             = 1;
	const double hyperparam_cost_ref_val_steering  = 500;
	const double hyperparam_cost_ref_val_throttle  = 10;
	const double hyperparam_cost_ref_seq_throttle  = 500;
	const double hyperparam_cost_ref_seq_steering  = 5000;  

#### 2.1.2 Kinematic model

	In this part, the car states is comprised of 6 elements:
	x = x coordonee of car position
	y = y coordonee of car position
	psi = car orientation
	v= car velocity
	cte = cross track error
	e_psi = error of car orientation

	and 2 outputs of MPC controller:
 
	delta = steering angle
	a = acceleration / throttle value

	And we are using Udacity's suggested kinematic model which is simple to be realized .
	The next time step(t+1) car states are directly calculated from present time step (t) car state,and continously N time step can be predicted in our project.

	x​t+1​​=x​t​​+v​t​​∗cos(ψ​t​​)∗dt 

	y​t+1​​ = y​t​​+v​t​​*sin(ψ​t​​)*dt

	ψ​t+1​​ = ψ​t​​+​L​f​​​​*v​t*δ*dt

	v​t+1​​ = v​t​​+a​t​∗dt

	cte​t + 1​​ = f(x​t​​) − y​t​​ + (v​t​​∗sin(eψ​t​​)∗dt)

	eψ​t+1​​ = ψ​t​​ − ψdes​t​​ + (​L​f​​​​ * v *​ t * δ​t​​ * dt)

### 2.2 Solve

	The solve function takes present time step car states and coeffs as input and returns the results of the COIN-OP solve(). The coeffs are of a polynomial fitted  reference way points line.

	By using COIN-OP Solve we can get the mathematical result of the problem  in the MPC.cpp

	vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)

	in this function, we call ipopt to solve this optimal algorithm with contraints :

	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);


	CppAD::ipopt::solve<Dvector, FG_eval>( options, vars, vars_lowerbound, 
                                         vars_upperbound, constraints_lowerbound,
                                         constraints_upperbound, fg_eval, solution);
										 
	And we take only the first time step output of delta and a  as our present controller output, it shows as below:

	// update the controller outputs of mpc class
	this->steering_angle = solution.x[delta_start] ;
	this->throttle       = solution.x[a_start] ;


## 3 Timestep Length and Elapsed Duration  (N & dt)
	N  stands for the number of forecasting steps .And  dt stands for the control time step which we calculate and output a control output every dt duration.
	and T=N*dt stands for the controller forcasting time duration.
	
	Because every ideal model has difference from the reality,the forecasting error will turn larger and larger as time goes by.
	So if we select T too big ,it will be no use but assuming more calculation.
	And if T is two little ,it will not benifit from the MPC precition ability.
	
	After trying 0.5,1.0 ,1.5, i finally choose T =1.0 seconds.
	
### 3.1  dt selection	

	dt is really impact much on the system.
	if the system states change rapidly,we must select a little dt to control the system in time .Otherwise the car is easy to drive out of road.
	But too little dt is not always good, which will assume more calculation and easy to make the car output change too quickly.
	Evenmore ,when dt<  system latency, it introduce much more prediction error and make the system unstable.
	
	i tried dt =0.05, when introduce latency ,the predicton is messed up and the  car is hard to control .
	At last ,i select dt =0.1 .
	
### 3.2 N selection
     In fact ,when T and dt are selected , N is fixed。
	 and i have tried N=7 which is ok but the predict green line seems short in the simulator.
	 
	
## 4 Polynomial Fitting and MPC Preprocessing
	i use polyfit() funtion to fit the way points into a polynomial line with order of 3.  
	and i transform the way points to car space coordinates before fitting into polyfit().

#### 4.1 car space transformation (in main.cpp)

	coeffs = polyfit(x_car_space, y_car_space, 3) ;

	x_car_space, y_car_space are reference way points coordonees in car space.

	In car coordinates x is positive dead ahead, and y is positive to the left of x. In map ordinates x is positive to the right and y is positive going up (like a normal graph). Transforming to car space will make the visualization more intutively and avoid the one x to mutiple y  situation when the fitting line is vertical in map ordinates .

          for (int i = 0;   i < ptsx.size() ;   i++) {
            x_car_space(i) = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi)  ;
            y_car_space(i) = (ptsy[i] - py) * cos(psi) - (ptsx[i] - px) * sin(psi)  ;
          }
		  here ptsx,ptsy are reference way points coordonees in map space.
                    
#### 4.2 State vector

	The state vector is defined in main.cpp main() as follows:
	in the car space,
                    px = 0;
                    py = 0;
                    psi = 0;

	We pass the coeffs of the fitted line to our evaluation functions, and the cte and car orientation error are calculated as follows:
           auto cte = polyeval(coeffs, px) - py;
           // calculate the orientation error
           auto f1=coeffs[1] + (2 * coeffs[2] * px) + (3 * coeffs[3] * (px * px));
           auto epsi = psi - atan(f1);

           Eigen::VectorXd state(6);
           state << px, py, psi, v, cte, epsi;

	
## 5 Model Predictive Control with Latency
	The latency is introduced to mimic real world driving, where actuators are limited by wire distance, mechanical properties and other variables, and so are unlikely to respond instantaneously.
	Besides, the controller calculation latency and presampling latency and other system lantency can also be taken into consideraton by MPC controller.
	
	So here i define the latency  as MPC class variable to simulate all the system lantency.
	And an lantency compensation is made in MPC.cpp before begining to calculate the control output, which coded in MPC.cpp as follows:
	
	 // latency compensation for the state calculation  in car space ordonates
    x+= v * latency ;
    epsi+=v * steering_angle/ Lf * latency ;  //previous_steering_angle
	cte+= v*CppAD::sin(epsi) *latency;
	
	where latency = 0.1  assumed in project.
	here ,steering_angle and throttle stands for the previous control output saved as the MPC class variable before the present control output update these values.
	By introducing latency compensation ,the MPC controller can overcome the error caused by system latency.
		

Circling back to the state vector, we define px = v * latency ; 

## 6 Road curvature and Ref_v

	In reality, human drive slow down when the road leads to a sharp turning, and speed up when the road is straight and no obstacle ahead.
	To mimic this human driving and make the car to handle sharp turning, i calculate the road curvature and simply set the car reference speed according to the curvature radias
	This part is implemented in main.cpp as follows:
	
	        auto road_curvature=pow(1.0+f1*f1,1.5)/fabs(f2);

            if(road_curvature<40) ref_v =30;
            else if(road_curvature<100 && road_curvature>40) ref_v =50;
            else ref_v =80;


## 7 Reflections

    So this is a very intuitive project to implement a simple MPC controller . The car can react quickly and smoothy on the track . And the MPC controller can predict the next several car states well. And the MPC controller can compensate the system lantency,which is not able to be handled in PID controller. In real life ,to make the model more accuate , we can introduce the tire model and even the road situation .
	And i feel a little unsatisfied to tune the hyperparameters manually,wich can be workable but not optimal globlely. I think that tuing in a nureaul network may be helpful.I will try it later.
    
    
