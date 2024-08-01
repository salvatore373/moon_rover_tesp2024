import math
def calculate_heading(x1, y1, x2, y2):
    # Compute the direction vector
    dx = x2 - x1
    dy = y2 - y1

    # Calculate the heading angle in radians
    theta_radians = math.atan2(dy, dx)

    # Convert the angle to degrees
    theta_degrees = math.degrees(theta_radians)

    return theta_degrees
    
class PDController: 
    def __init__(self, Kpx, Kdx, Kpy, Kdy, Kp_theta, Kd_theta, Kv, Komega, d):
        self.Kpx = Kpx
        self.Kdx = Kdx
        self.Kpy = Kpy
        self.Kdy = Kdy
        self.Kp_theta = Kp_theta
        self.Kd_theta = Kd_theta
        self.Kv = Kv
        self.Komega = Komega
        self.d = d
        self.prev_ex = 0
        self.prev_ey = 0
        self.prev_e_theta = 0

    def compute_control(self, x_d, y_d, theta_d, x, y, theta, dt): 
        # Calculate position errors 
        ex = x_d - x
        ey = y_d - y 
        # Calculate heading error 
        e_theta = theta_d - theta 
        # Derivative of errors 
        de_x = (ex - self.prev_ex) / dt 
        de_y = (ey - self.prev_ey) / dt 
        de_theta = (e_theta - self.prev_e_theta) / dt 
        # PD control for position 
        ux = self.Kpx * ex + self.Kdx * de_x 
        uy = self.Kpy * ey + self.Kdy * de_y
        # PD control for heading 
        u_theta = self.Kp_theta * e_theta + self.Kd_theta * de_theta 
        # Convert to wheel velocities 
        v = self.Kv * (ux + uy) 
        omega = self.Komega * u_theta 
        v_FL = v - (self.d / 2) * omega 
        v_FR = v + (self.d / 2) * omega 
        v_R = v 
        # Update previous errors 
        self.prev_ex = ex 
        self.prev_ey = ey 
        self.prev_e_theta = e_theta 
        return v_FL, v_FR, v_R 

"""    
# Initialize PDController with suitable gains and parameters  
controller = PDController(Kpx=1, Kdx=0.1, Kpy=1, Kdy=0.1, Kp_theta=1, Kd_theta=0.1, Kv=1, Komega=1, d=0.15) 
# Get control commands 
v_FL, v_FR, v_R = controller.compute_control(x_d=1, y_d=1, theta_d=0, x=0, y=0, theta=0, dt=0.1) 
print("Front Left motor velocity:", v_FL)
print("Front Right motor velocity:", v_FR)
print("Rear motor velocity:", v_R)
"""