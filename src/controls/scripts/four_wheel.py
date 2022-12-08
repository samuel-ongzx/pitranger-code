class FourWheel:
    def __init__(self, w, l, h, r, vxl=0, vxr=0):
        '''
        This initializes the four wheeled kinematic model using wheel jacobians
        '''
        # width of the vehicle from the center frame (total vehicle is 2w)
        self.width         = w
        # length of vehicle from center frame (total vehicle is 2l)
        self.length        = l
        self.height        = h
        self.wheel_radius  = r
        # velocity constraints for the contact points, this is a knob to tune slip estimate
        # self.v_constraints_l = np.zeros((12, 1))
        # self.v_constraints_r = np.zeros((12, 1))
        self.v_constraints = np.zeros((12, 1))
        self.v_constraints[[0, 6]]  = vxl
        self.v_constraints[[3, 9]]  = vxr

        self.jacobian      = np.array(
            [
                [0    , -h-r  , -w  , 1  , 0  , 0  , -r  , 0   , 0  , 0]  ,
                [h+r  , 0     , l   , 0  , 1  , 0  , 0   , 0   , 0  , 0]  ,
                [w    , -l    , 0   , 0  , 0  , 1  , 0   , 0   , 0  , 0]  ,
                [0    , -h-r  , w   , 1  , 0  , 0  , 0   , -r  , 0  , 0]  ,
                [h+r  , 0     , l   , 0  , 1  , 0  , 0   , 0   , 0  , 0]  ,
                [-w   , -l    , 0   , 0  , 0  , 1  , 0   , 0   , 0  , 0]  ,
                [0    , -h-r  , -w  , 1  , 0  , 0  , 0   , 0   , -r , 0]  ,
                [h+r  , 0     , -l  , 0  , 1  , 0  , 0   , 0   , 0  , 0]  ,
                [w    , l     , 0   , 0  , 0  , 1  , 0   , 0   , 0  , 0]  ,
                [0    , -h-r  , w   , 1  , 0  , 0  , 0   , 0   , 0  , -r] ,
                [h+r  , 0     , -l  , 0  , 1  , 0  , 0   , 0   , 0  , 0]  ,
                [-w   , l     , 0   , 0  , 0  , 1  , 0   , 0   , 0  , 0]
            ]
        )
        # slice up the jacobian
        self.body_jacobian    = self.jacobian[:, :6]
        self.wheel_jacobian  = self.jacobian[:, 6:]
        self.inv_wheel_jacobian = np.array(
            [
                [-1/r  , 0  , 0  , 0     , 0  , 0  , 0     , 0  , 0  , 0     , 0  , 0 ]  ,
                [0     , 0  , 0  , -1/r  , 0  , 0  , 0     , 0  , 0  , 0     , 0  , 0 ]  ,
                [0     , 0  , 0  , 0     , 0  , 0  , -1/r  , 0  , 0  , 0     , 0  , 0 ]  ,
                [0     , 0  , 0  , 0     , 0  , 0  , 0     , 0  , 0  , -1/r  , 0  , 0 ]

            ]
        )

        # predefine some things for ease
        a = 1/(w)
        b = 1/(l)
        c = w/((l**2 + w**2))
        d = l/((l**2 + w**2))
        e = (r*h**2 + h*r**2)/(h*r*l)
        f = (r*h**2 + h*r**2)/(h*r*w)
        self.inv_body_jacobian = 1/4 * np.array(
            [
                [0  , 0 , a  , 0 , 0 , -a , 0  , 0  , a  , 0 , 0  , -a] ,
                [0  , 0 , -b , 0 , 0 , -b , 0  , 0  , b  , 0 , 0  , b]  ,
                [-c , d , 0  , c , d , 0  , -c , -d , 0  , c , -d , 0]  ,
                [1  , 0 , -e , 1 , 0 , -e , 1  , 0  , e  , 1 , 0  , e]  ,
                [0  , 1 , -f , 0 , 1 , f  , 0  , 1  , -f , 0 , 1  , f]  ,
                [0  , 0 , 1  , 0 , 0 , 1  , 0  , 0  , 1  , 0 , 0  , 1]
            ]
        )

    def actuation(self, body_velocity):
        '''
        Perform the actuation kinematics: body -> joint. 
        Output is in radians per second
        '''
        if not isinstance(body_velocity, np.ndarray):
            body_velocity = np.array(body_velocity)
            body_velocity = body_velocity.reshape((-1, 1))
        assert body_velocity.shape == (6, 1)

        # if (body_velocity[2] <= 0):
        #     v_constraint = self.v_constraints_l
        # elif (body_velocity[2] > 0):
        #     v_constraint = self.v_constraints_r
        # else:
        #     v_constraint = np.zeros((12,1))

        return np.matmul(self.inv_wheel_jacobian, (self.v_constraints - np.matmul(self.body_jacobian, body_velocity)))
    
    def navigation(self, wheel_velocity):
        '''
        Perform the navigation kinematics: joint -> body.
        Output is in meters per second
        '''
        if not isinstance(wheel_velocity, np.ndarray):
            wheel_velocity = np.array(wheel_velocity)
            wheel_velocity = wheel_velocity.reshape((-1, 1))
        assert wheel_velocity.shape == (4, 1)

        # v_constraints = np.zeros((12,1))
        # v_constraints = self.v_constraints

        # print(np.matmul(self.inv_body_jacobian, (self.v_constraints - np.matmul(self.wheel_jacobian, wheel_velocity)))[3], 
        #       np.matmul(self.inv_body_jacobian, (v_constraints - np.matmul(self.wheel_jacobian, wheel_velocity)))[3])
        return np.matmul(self.inv_body_jacobian, (self.v_constraints - np.matmul(self.wheel_jacobian, wheel_velocity)))

    def set_slip_constraints(self, wheelsInput):
        # check zero
        # print(wheelsInput)
        if all(i == 0 for i in wheelsInput):
          return 0, 0

        # Wheel Radius
        radius = 0.0955

        # Baseline Slip	      2%,     3%,     6%,    8%,    12%,    16%,    20%
        speed = np.array([0.0,     0.02,     0.03,     0.04,     0.05,     0.06,     0.07])  # m/s
        slip  = np.array([0.0, 0.001024, 0.001626, 0.002288, 0.003015, 0.003798, 0.004641])# slip in m/s
        # slip *= 1.1

		    # Drive Arc Slip  1.0m,   1.5m,   2.0m,   2.5m,   3.0m,     ?m,   8.0m  
        # turn    = np.array([2.0000, 1.5466, 1.3837, 1.2958, 1.2404, 1.1000, 1.0000]) # drive ratio
        # factor  = np.array([  2.00,   1.80,   1.60,   1.50,   1.40,   1.20,   1.00]) # factor 

        # Drive Arc Slip            1.0m,   1.5m,   2.0m,   2.5m,   3.0m,   8.0m  
        turn_rad = np.array([   0.5,   1.0,    1.5,    2.0,    2.5 ,   3.0,   8.0])
        turn     = np.array([ 2.000, 1.857, 1.5000, 1.3529, 1.2727, 1.2222, 1.000]) # drive ratio
        factor   = np.array([  2.00,  1.90,   1.65,   1.55,   1.50,   1.40,  1.00]) # factor 
        factor *= 1.1

        
        # Clip wheel rates
        wheelsInput = np.clip(wheelsInput, -0.7, 0.7)

        speedLeft  = (wheelsInput[0] + wheelsInput[2]) * radius / 2.0
        speedRight = (wheelsInput[1] + wheelsInput[3]) * radius / 2.0
        drive_ratio = max(np.abs(speedLeft), np.abs(speedRight)) / min(np.abs(speedLeft), np.abs(speedRight))
        slipFactor = 1.0

        # Interpolate to get slip values
        f_slip = interpolate.interp1d(speed, slip)
        slipLeft  = f_slip(np.abs(speedLeft))
        slipRight = f_slip(np.abs(speedRight))

        # 2nd Layer 
        if np.sign(speedLeft): 
          slipLeft *= -1.0
        if np.sign(speedRight):
          slipRight *= -1.0

        # Interpolate for 2nd wave of slip params due to drive arc
        if (drive_ratio > 1.0 and drive_ratio < 2.0):
          f_factor = interpolate.interp1d(turn, factor)
          slipFactor = f_factor(abs(drive_ratio))
          slipLeft *= slipFactor
          slipRight *= slipFactor

        self.v_constraints[[0, 6]]  = slipLeft
        self.v_constraints[[3, 9]]  = slipRight
        # print(slipLeft, slipRight)

        return slipLeft, slipRight

    def set_acutation_slip_constraints(self, speed, radius):

        # Wheel Radius
        radius = 0.0955

        # Baseline Slip	      2%,     3%,     6%,    8%,    12%,    16%,    20%
        speed = np.array([0.0,     0.02,     0.03,     0.04,     0.05,     0.06,     0.07])  # m/s
        slip  = np.array([0.0, 0.001024, 0.001626, 0.002288, 0.003015, 0.003798, 0.004641])# slip in m/s

        # Drive Arc Slip            1.0m,   1.5m,   2.0m,   2.5m,   3.0m,   8.0m  
        turn    = np.array([ 2.000, 1.857, 1.5000, 1.3529, 1.2727, 1.2222, 1.000]) # drive ratio
        factor  = np.array([  2.00,  1.90,   1.65,   1.55,   1.50,   1.40,  1.00]) # factor 
        factor *= 1.1

        # Interpolate to get slip values
        f_slip = interpolate.interp1d(speed, slip)
        slipLeft  = f_slip(np.abs(speedLeft))
        slipRight = f_slip(np.abs(speedRight))


    def reset_slip_constraints(self):
        self.v_constraints = np.zeros((12,1))
