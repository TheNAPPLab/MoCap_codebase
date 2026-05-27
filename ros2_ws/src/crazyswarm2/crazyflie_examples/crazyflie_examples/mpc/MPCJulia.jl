module MPCJulia
# using ControlSystems # currenly not using this package, but it can be used for linear systems and LQR problems; for nonlinear MPC, we are using JuMP and Ipopt
using LinearAlgebra
using StaticArrays
using JuMP
using Plots
using Ipopt


#define the dynamics of the quadrotor 

function continious_dynamics(x::Vector{Float64}, u::Vector{Float64})

    #differentiation of the dynamics to define the physics of the system, how the system "behaves" and "evolves" over time,

    #=
    x_dot = u[1] * cos(x[3]) # velocity*direction of x position
    y_dot = u[1] * sin(x[3]) #velocity*direction of y position
    yaw_dot = u[2]           #angular velocity
    =#

    #NEW: Converting from 2D to 3D, we are adding the z position and velocity as well
    x_dot = u[1]   # vx in world frame
    y_dot = u[2]   # vy in world frame  
    z_dot = u[3]   # vz in world frame
    yaw_dot = u[4] # yaw rate


    zeta_dot = [x_dot; y_dot; z_dot; yaw_dot] #differentiation of the velocity is basically accelaration,which is omega
    #B = @SMatrix [] 
    return zeta_dot
end

#For this module, we are keeping it nonlinear

function discrete_dynamics(x::Vector{Float64}, u::Vector{Float64}, Ts::Float64)
    """
    We are using the forward Eular for the discritization:
        x[n+1] = x[n] + Ts * f(x[n],u[n])
    """
    zeta_dot = continious_dynamics(x, u)
    zeta_next = x + Ts * zeta_dot
    println("zeta_next: $zeta_next")
    return zeta_next
end





#Definition of the structure of the MPC controller  
mutable struct NonlinearMPCController
    # System parameters
    Ts::Float64          # sampling time
    N::Int              # prediction horizon
    nx::Int             # number of states
    nu::Int             # number of inputs

    # Cost matrices for the objective function
    Q::Matrix{Float64}   # state cost
    R::Matrix{Float64}   # control cost


    # Constraints
    u_min::Vector{Float64}  # input lower bounds
    u_max::Vector{Float64}  # input upper bounds


    # Solver
    model::Model
end

function referencetrajectory(x_initial,x_target, N, Ts)
    x_ref = zeros(4, N + 1)
    u_ref = zeros(4, N)

    ref_velocity = 0.2
    for i in 1:(N+1)
        penalty = (i - 1)/N
        x_ref[:, i] = ((1-penalty)*x_initial ) + penalty * x_target

        #println("time $i = $x_ref\n")
        #x_target = x_ref[:, i]
    end


    return x_ref



end


#Here, we are designing the MPC controller based on JuMP
#As we are using the IpOpt solver which uses the modeling language JuMP; 
#we are defining the parameters and variables as per JuMP syntax for the convention of the solver
#function NonlinearMPCController(x_ref,x_initial,N, Ts)
#function NonlinearMPCController()
function NonlinearMPCController(x_initial::Vector{Float64}, x_target::Vector{Float64})
    N=20
    Ts=0.1
    nx, nu = 4, 4 #number of states, number of control inputs

    #=
    Q = Diagonal([1.0, 1.0, 1.0])      # position and yaw angle weight 
    R = Diagonal([0.1, 0.1])           # control penalties
    =#
    # NEW: Converting from 2D to 3D, we are adding the z position and velocity as well, so we need to update the cost matrices as well
    # Will tune these parameters later, but for now we are just putting some random values to see how the controller works
    Q = Diagonal([1.0, 1.0, 20.0, 1.0])  # equal weight on x, y, z, yaw tracking
    R = Diagonal([0.1, 0.1, 0.1, 0.1])  # penalize large velocities


    # Default constraints bounds for control
    # u_min = [-2.0, -π / 2]  # [min velocity, min angular velocity]
    # u_max = [2.0, π / 2]    # [max velocity, max angular velocity]

    # NEW: Converting from 2D to 3D, we are adding the z position and velocity as well, so we need to update the control input constraints as well
    u_min = [-0.6, -0.6, -0.3, -pi/4]  # [min vx, min vy, min vz, min yaw rate]
    u_max = [0.6, 0.6, 0.3, pi/4]    # [max vx, max vy, max vz, max yaw rate]

    #=
    x_initial = [0.0, 0.0, 0.0, 0.0] # Initial x_pos, y_pos and yaw 
    x_target = [0.0, 0.0, 0.0, 0.0]    #The end position for the drone, we are keeping it same as the initial position for now, but we can change it to see how the controller works with different target positions
    =#

    x_ref = referencetrajectory(x_initial,x_target,N,Ts)
    #Initial_state and control

    #First, we need to initialize the solver model that we are using.
    #We will be using it when we are defining the variables and constraints
    #to point which solver model we are using.
    model = Model(Ipopt.Optimizer)
    #set_optimizer_attribute(model, "print_level", 0)
    set_optimizer_attribute(model, "max_iter", 20)

    #Define the variables in the JuMP formulation

    @variable(model, x[1:nx, 1:N+1]) # states (columns) x timesteps(rows); N+1 because we are always predicting the next time step
    @variable(model, u[1:nu, 1:N])


    #The initial state of the robot is being sent form the main function where we are defining the initial position
    # and the target position that the robot needs to go to

    @constraint(model, x[:,1] == x_initial)
    # @constraint(model, x[:,N+1] == x_target)
    #x[:,1] == x_initial
    # temp = value(x[:,1])
    # println("Temp::::$temp")

    #Note: there cannot be any space after constraints!!!

    #We need to declare all the input as a constraint, which is basically a JuMP convention;
    #As we already have the discritized verison of the dynamics, we need to put it in a loop
    #and treat it as a constraint with the variables that we have in the controller function
    #=
    for n in 2:N-1
        @constraint(model, x[1,n+1] == x[1,n] +Ts * u[1,n] * cos(x[3,n]) )
        # x_val = value.(x[1,n+1])
        # println("$x_val")  #In the dynamics discritization, we are basically calculating the next state
        @constraint(model, x[2,n+1] == x[2,n] +Ts * u[1,n] * sin(x[3,n]) )
        @constraint(model, x[3,n+1] == x[3,n] +Ts * u[2,n])
       @constraint(model, u_min[1] <= u[1,n] <= u_max[1])
       @constraint(model, u_min[2] <= u[2,n] <= u_max[2])
    end
    =#

    #NEW: Converting from 2D to 3D, we are adding the z position and velocity as well, so we need to update the constraints for the dynamics as well
    # for n in 2:N-1
    for n in 1:N
        @constraint(model, x[1,n+1] == x[1,n] + Ts * u[1,n])
        @constraint(model, x[2,n+1] == x[2,n] + Ts * u[2,n])
        @constraint(model, x[3,n+1] == x[3,n] + Ts * u[3,n])
        @constraint(model, x[4,n+1] == x[4,n] + Ts * u[4,n])
        @constraint(model, u_min[1] <= u[1,n] <= u_max[1])
        @constraint(model, u_min[2] <= u[2,n] <= u_max[2])
        @constraint(model, u_min[3] <= u[3,n] <= u_max[3])
        @constraint(model, u_min[4] <= u[4,n] <= u_max[4])
        @constraint(model, x[3,n] >= 0.05) # Ensure the drone stays above the ground
    end
    @constraint(model, x[3, N+1] >= 0.05) # Ensure the final position is also above the ground
    # @constraint(model, x[3, N+1] == x_target[3]) # Ensure the final z position matches the target z position

    #x0 = [0.0, 0.0, 0.0] #Initial state for the robot 




    #Objective function : total cost(x,u) = statecost(x) + controlcost(u)
    state_cost = 0.0
    control_cost = 0.0
    total_cost = 0.0

    # #Constraints
    # for i in 1:N
    #     u_constrained[i] = clamp(u[i], u_min[i], u_max[i])
    # end

    #We have the difference between the reference and the current state as a L2 norm squared 
    #We have taken the diagonal weighted matrix for the state and control cost, which makes it easier to track the error for 
    #the different parts of the state(x_pos,y_pos,yaw)
    #=
    for k in 1:N-1

        for i in 1:nx
            state_cost += Q[i,i] * (x_ref[i,k] - x[i,k])^2  
            #state_cost += (x_ref[i,k] - x[i,k])^2 
        
        end
        for i in 1:nu
        #     #control_cost += R[i,i] * (u[i,k] - u[i,k-1])^2
             control_cost += transpose(u[i,k]) * R[i,i]  * u[i,k]
        end

        total_cost += state_cost + control_cost
    end
    =#

    for k in 1:N
        step_state_cost = 0.0
        step_control_cost = 0.0
        for i in 1:nx
            step_state_cost += Q[i,i] * (x_ref[i,k] - x[i,k])^2
        end
        for i in 1:nu
            step_control_cost += transpose(u[i,k]) * R[i,i] * u[i,k]
        end
        total_cost += step_state_cost + step_control_cost
    end

    terminal_weight = 20.0
    terminal_cost =0.0
    for i in 1:nx
        terminal_cost += terminal_weight * Q[i,i] *(x_target[i]-x[i,N+1])^2
        #terminal_cost +=(x_target[i]-x[i,N+1])^2
        
    end

    total_cost += terminal_cost

    #Objective function
    @objective(model,Min,total_cost)
    optimize!(model)
    solution_summary(model)

    println("""
    termination_status = $(termination_status(model))
    primal_status      = $(primal_status(model))
    objective_value    = $(objective_value(model))
    """)
    assert_is_solved_and_feasible(model)

    

     x_actual = value.(x)
     u_actual = value.(u)
    #return value.(x)
    
        #plotting
    println("Control points:u1= $u_actual[1],\n u2= $u_actual[2]")
    return x_actual

    #use the IPOPT solver to optimize



end




#Generate reference trajectory which has a constant target


function plotting(x_ref,x_actual,Ts, N)

    

    x_pos = x_ref[1, :]
    y_pos = x_ref[2, :]
    z_pos = x_ref[3, :]
    yaw = x_ref[4, :]
    time = 0:Ts:(N*Ts)


    println("x_position: $x_pos")


    x_pos1 = x_actual[1, :]
    y_pos1 = x_actual[2, :]
    z_pos1 = x_actual[3, :]
    yaw1 = x_actual[4, :]
    time = 0:Ts:(N*Ts)
    println("x1_position: $x_pos1")

    # x_pos = x_ref[1, :]
    # y_pos = x_ref[2, :]
    # yaw = x_ref[3, :]
    # time = 0:Ts:(N*Ts)

    #println("$x_pos")

    point = sqrt.(x_pos.^2 +y_pos.^2 +yaw.^2)
    #print("$point")

    p = plot(time, x_pos,  
              label = "reference",
              xlabel="Time (s)",
              ylabel="State Value",
              title="Reference vs. Calculated Trajectory",
              linewidth=2,
              marker=:circle,
              markersize=4,
              grid=true)
    
        #display(p)


    # point1 =sqrt.(x_pos1.^2 +y_pos1.^2 +yaw1.^2)
    # #print("$point")

    p1 = plot!(p,time,x_pos1,
            label="calculated",
            linewidth=2,
            marker=:circle,
            markersize=4)

    # p1 = plot!(time, x_pos1, p, 
    #           #label = "Trajectory tracking",
    #           xlabel="Time (s)",
    #           ylabel="x_position",
    #           title="x_position vs Time",
    #           linewidth=2,
    #           marker=:circle,
    #           markersize=4,
    #           grid=true)


    p_y = plot(time, y_pos,  
              
              xlabel="Time (s)",
              ylabel="State Value",
              title="Reference vs. Calculated Trajectory",
              linewidth=2,
              marker=:circle,
              markersize=4
              )

   p2 = plot!(p_y,time, y_pos1,  
              #label = "Trajectory tracking",
              
              linewidth=2,
              marker=:circle,
              markersize=4
              )



   p_yaw = plot(time, yaw,  
              #label = "Trajectory tracking",
              xlabel="Time (s)",
              ylabel="yaw",
              title="yaw vs Time",
              linewidth=2,
              marker=:circle,
              markersize=4,
              grid=true)


   p3 = plot!(p_yaw,time, yaw1,  
              #label = "Trajectory tracking",
              
              linewidth=2,
              marker=:circle,
              markersize=4
              )


    p_z = plot(time, z_pos,
          xlabel="Time (s)",
          ylabel="z position (m)",
          title="Z Position vs Time",
          linewidth=2,
          marker=:circle,
          markersize=4,
          grid=true)

    p4 = plot!(p_z, time, z_pos1,
          linewidth=2,
          marker=:circle,
          markersize=4)
        
    p_total = plot(p1, p2, p4, p3, layout = grid(4, 1))
            #label="Calculated Trajectory",
            #linewidth=2,
            #marker=:square,
            #markersize=4)
    
    #     display(p1)




end
#Writing down a simple solver to see how controller works with optimization
function TestFunction()

    println("Examples!!!!")
    Ts = 0.1
    N = 20

    #Initial parameters

    #x_initial = [0.0, 0.0, 0.5, 0.0] # Initial x_pos, y_pos and yaw 
    #x_target = [0.0, 0.0, 0.5, 0.0]    #The end position for the 

    # Test to see how the controller works with different target positions, we are changing the target position to (1,1) from (0,0)
    x_initial = [0.0, 0.0, 0.5, 0.0]
    x_target = [1.0, 1.0, 0.5, 0.0]

    x_ref = referencetrajectory(x_initial,x_target,N,Ts)


    x_actual = NonlinearMPCController(x_initial, x_target)

    traj_plotting = plotting(x_ref,x_actual,Ts,N)

    # Display the plot and save it as a png file
    display(traj_plotting)
    savefig(traj_plotting, "mpc_test.png")


    return traj_plotting
    
    #optimization = NonlinearMPCController(x_ref,x_initial,N, Ts)
    #println("Optimization result = $optimization")


end



#state vector - [position x, position y , yaw angle]

# x = [0, 1.5, 2.5, 3.5]
# y = [0, 0.5, 3.5, 4.5]
# yaw = [30, 20, 10, 25]

# #Control input - angular velocity
# v = [1.1, 5.9, 4.5, 7.9] #constant linear velocity
# omega = [1.0, 2.5, 3.2, 4.5] #constant angular velocity



#zeta =@SMatrix [x ; y ; yaw]

#We need to discretize the dynamics from the continious Time
#Ts = 0.1  #sample time
#N = 10 #Optimization Horizon

end


#Testing out how the solver with a really simple problem 
