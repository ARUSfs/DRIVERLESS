import numpy as np
import rospy
from fssim_common.msg import State
from common_msgs.msg import Map
from visualization_msgs.msg import Marker
import time


# <------------------------- EKF SLAM --------------------------------->
# ---> Robot Parameters
n_state = 3 # Number of state variables


#print(landmarks)
#print(n_landmarks)

# ---> Noise parameters
R = np.diag([0.002,0.002,0.0005]) # sigma_x, sigma_y, sigma_theta
Q = np.diag([0.003,0.005]) # sigma_r, sigma_phi

# ---> Helpful matrix
Fx = np.eye(3) # Used in both prediction and measurement updates
prev_t = time.time()

# ---> Measurement function
def sim_measurement(x,landmarks):

    rx, ry, rtheta = x[0], x[1], x[2]
    zs = [] # List of measurements
    for (lidx,landmark) in enumerate(landmarks): # Iterate over landmarks and indices
        lx,ly = landmark
        dist = np.linalg.norm(np.array([lx-rx,ly-ry])) # distance between robot and landmark
        phi = np.arctan2(ly-ry,lx-rx) - rtheta # angle between robot heading and landmark, relative to robot frame
        phi = np.arctan2(np.sin(phi),np.cos(phi)) # Keep phi bounded, -pi <= phi <= +pi
        if dist<robot_fov: # Only append if observation is within robot field of view
            zs.append((dist,phi[0],lidx))
    return zs

# ---> EKF SLAM steps
def prediction_update(mu,sigma,u):
    dt = time.time()-prev_t
    prev_t = time.time()
    
    rx,py,theta = mu[0],mu[1],mu[2]
    v,w = u[0],u[1]

    # Update state estimate mu with model
    state_model_mat = np.zeros((n_state,1)) # Initialize state update matrix from model
    state_model_mat[0] = -(v/w)*np.sin(theta)+(v/w)*np.sin(theta+w*dt) if w>0.01 else v*np.cos(theta)*dt # Update in the robot x position
    state_model_mat[1] = (v/w)*np.cos(theta)-(v/w)*np.cos(theta+w*dt) if w>0.01 else v*np.sin(theta)*dt # Update in the robot y position
    state_model_mat[2] = w*dt # Update for robot heading theta
    mu = mu + np.matmul(np.transpose(Fx),state_model_mat) # Update state estimate, simple use model with current state estimate

    # Update state uncertainty sigma
    state_jacobian = np.zeros((3,3)) # Initialize model jacobian
    state_jacobian[0,2] = (v/w)*np.cos(theta) - (v/w)*np.cos(theta+w*dt) if w>0.01 else -v*np.sin(theta)*dt # Jacobian element, how small changes in robot theta affect robot x
    state_jacobian[1,2] = (v/w)*np.sin(theta) - (v/w)*np.sin(theta+w*dt) if w>0.01 else v*np.cos(theta)*dt # Jacobian element, how small changes in robot theta affect robot y
    G = np.eye(sigma.shape[0]) + np.transpose(Fx).dot(state_jacobian).dot(Fx) # How the model transforms uncertainty
    sigma = G.dot(sigma).dot(np.transpose(G)) + np.transpose(Fx).dot(R).dot(Fx) # Combine model effects and stochastic noise

    print(mu)
    print(sigma)

    return mu,sigma

def measurement_update(mu,sigma,zs):

    rx,ry,theta = mu[0,0],mu[1,0],mu[2,0] # robot 
    delta_zs = [np.zeros((2,1)) for lidx in range(n_landmarks)] # A list of how far an actual measurement is from the estimate measurement
    Ks = [np.zeros((mu.shape[0],2)) for lidx in range(n_landmarks)] # A list of matrices stored for use outside the measurement for loop
    Hs = [np.zeros((2,mu.shape[0])) for lidx in range(n_landmarks)] # A list of matrices stored for use outside the measurement for loop
    for z in zs:
        (dist,phi,lidx) = z
        mu_landmark = mu[n_state+lidx*2:n_state+lidx*2+2] # Get the estimated position of the landmark
        if np.isnan(mu_landmark[0]): # If the landmark hasn't been observed before, then initialize (lx,ly)
            mu_landmark[0] = rx + dist*np.cos(phi+theta) # lx, x position of landmark
            mu_landmark[1] = ry+ dist*np.sin(phi+theta) # ly, y position of landmark
            mu[n_state+lidx*2:n_state+lidx*2+2] = mu_landmark # Save these values to the state estimate mu
        delta  = mu_landmark - np.array([[rx],[ry]]) # Helper variable
        q = np.linalg.norm(delta)**2 # Helper variable

        dist_est = np.sqrt(q) # Distance between robot estimate and and landmark estimate, i.e., distance estimate
        phi_est = np.arctan2(delta[1,0],delta[0,0])-theta; phi_est = np.arctan2(np.sin(phi_est),np.cos(phi_est)) # Estimated angled between robot heading and landmark
        z_est_arr = np.array([[dist_est],[phi_est]]) # Estimated observation, in numpy array
        z_act_arr = np.array([[dist],[phi]]) # Actual observation in numpy array
        delta_zs[lidx] = z_act_arr-z_est_arr # Difference between actual and estimated observation

        # Helper matrices in computing the measurement update
        Fxj = np.block([[Fx],[np.zeros((2,Fx.shape[1]))]])
        Fxj[n_state:n_state+2,n_state+2*lidx:n_state+2*lidx+2] = np.eye(2)
        H = np.array([[-delta[0,0]/np.sqrt(q),-delta[1,0]/np.sqrt(q),0,delta[0,0]/np.sqrt(q),delta[1,0]/np.sqrt(q)],\
                      [delta[1,0]/q,-delta[0,0]/q,-1,-delta[1,0]/q,+delta[0,0]/q]])
        H = H.dot(Fxj)
        Hs[lidx] = H # Added to list of matrices
        Ks[lidx] = sigma.dot(np.transpose(H)).dot(np.linalg.inv(H.dot(sigma).dot(np.transpose(H)) + Q)) # Add to list of matrices

    # After storing appropriate matrices, perform measurement update of mu and sigma
    mu_offset = np.zeros(mu.shape) # Offset to be added to state estimate
    sigma_factor = np.eye(sigma.shape[0]) # Factor to multiply state uncertainty
    for lidx in range(n_landmarks):
        mu_offset += Ks[lidx].dot(delta_zs[lidx]) # Compute full mu offset
        sigma_factor -= Ks[lidx].dot(Hs[lidx]) # Compute full sigma factor
        
    mu = mu + mu_offset # Update state estimate
    sigma = sigma_factor.dot(sigma) # Update state uncertainty
    return mu,sigma
# <------------------------- EKF SLAM --------------------------------->

# <------------------------- ROS --------------------------------->
uk = np.array([0.,0.]) 
def get_uk(msg):
    global uk
    uk = np.array([0.,0.]) 
    v = np.linalg.norm([msg.vx, msg.vy])
    r = msg.r
    uk = np.array([v,r])
    return uk 

def gen_archivo(mu):
    nombre_archivo = "state_save.txt"
    with open(nombre_archivo, 'w') as archivo:
        archivo.write(str(mu))

def state_marker(optimal_state_estimate_k, state_pub):
    marker = Marker()
    marker.header.frame_id = 'fssim/vehicle/cog'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.SPHERE 
    marker.action = Marker.MODIFY
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.position.x = optimal_state_estimate_k[0]
    marker.pose.position.y = optimal_state_estimate_k[1]
    marker.pose.position.z = 0.0

    state_pub.publish(marker)

# <------------------------- ROS --------------------------------->

def main():

    global mu, sigma, uk
    
    # Initialize robot and discretization time step
    mu = np.array([0,0,0]) # px, py, theta

    # Initialize robot state estimate and sigma
    sigma = np.eye((n_state,n_state))*0.01
    sigma[2,2] = 0

    rospy.init_node('slam', anonymous=True)

    rospy.Subscriber('/fssim/base_pose_ground_truth', State, get_uk, queue_size=1)

    state_pub = rospy.Publisher("/state_marker", Marker, queue_size=1)

    try: 
        while not rospy.is_shutdown():

            # Get measurements
           
            # EKF Slam Logic
            mu, sigma = prediction_update(mu,sigma,uk) # Perform EKF prediction update
            mu, sigma = measurement_update(mu,sigma,zs) # Perform EKF measurement update

            state_marker(mu, state_pub)

            rospy.sleep(0)

    except KeyboardInterrupt:
        gen_archivo(mu)

    rospy.spin()

if __name__ == '__main__':
    main()