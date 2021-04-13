import numpy as np
import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt
np.set_printoptions(precision=3)

PLANE_H = 0.03
Y_OBS = np.clip(np.random.normal(loc=0., scale=.2, size=2), -0.2, 0.2)
#OBS = [[0.45, Y_OBS[0], PLANE_H], [1., Y_OBS[1], PLANE_H]]
OBS = [[0.45, 0.2, PLANE_H], [1., 0, PLANE_H]]
ROBOT_MASS = 0.1


class Env:
    def __init__(self, with_obs=False):
        # initialize the simulator and blocks
        self.physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF('plane.urdf', useFixedBase=True)
        p.changeDynamics(plane_id, -1, lateralFriction=0.99)

        
        p.setGravity(0, 0, 0)

        # add obstacles
        self.with_obs = with_obs
        if with_obs:
            for obs in OBS:
                self.obs = p.loadURDF('cylinder_obs.urdf',
                                      basePosition=obs,
                                      useFixedBase=True)

        # add robot
        if with_obs:
            y0 = 0.
            x0 = 1.75
        else:
            y0 = np.random.randn()
            x0 = np.random.randn()

        self.rob = p.loadURDF('cylinder_robot.urdf',
                              basePosition=[x0, y0, PLANE_H],
                              useFixedBase=False)

    def simulate(self, max_t=600):
        for sim_step in range(max_t):
            q, v = self.get_state()
            u = dmp_control(q, v)
            if self.with_obs:
                u = u + collision_avoidance(q, v)
            self.apply_control(u)
            p.stepSimulation()
            # print(self.get_state())
            time.sleep(0.01)

    def get_state(self):
        q, ori = p.getBasePositionAndOrientation(self.rob)
        v, w = p.getBaseVelocity(self.rob)

        return np.asarray(q[:2]), np.asarray(v[:2])

    def apply_control(self, u):
        link_id = -1
        force_loc = np.array([0., 0., 0.])

        u_3 = np.append(u, 0.)

        p.applyExternalForce(self.rob,
                             link_id,
                             u_3,
                             force_loc,
                             p.LINK_FRAME)


def dmp_control(q, v, qd=np.zeros((2, )), vd=np.zeros((2, ))):
    """
        The DMP controller
        :param q: np.array((2, )) -- configuration of the robot
        :param v: np.array((2, )) -- velocity of the robot
        :param qd: np.array((2, )) -- desired configuration of the robot
        :param vd: np.array((2, )) -- desired velocity of the robot
        :return: u: np.array((2, )) -- DMP control
    """
    print(q,v)
    u = np.zeros(2)
    # m = ROBOT_MASS
    # ax, ay = 0, 0
    kp = 5
    kv = 2
    #########################################
    gx, gy = qd[0], qd[1]
    # dotx, doty = vd[0], vd[1] 
    
    ux = kp*(gx-q[0])-kv*(-vd[0]+v[0])
    uy = kp*(gy-q[1])-kv*(-vd[1]+v[1])
    #########################################
    u[0] = ux
    u[1] = uy
    return u


def collision_avoidance(q, v, qd=np.array([0., 0.])):
    
    u = np.zeros(2)
    gamma = 150.
    beta = 5
    theta = np.pi/2
    obs = np.array(OBS)
    xy = obs[:,0:2]
    u_DMP = dmp_control(q, v, qd)
    u_COLLAVOID = np.zeros((len(xy),2))
    for i in range(len(xy)):
        o_x = xy[i]-q
        r = np.cross(o_x,v)
        R = np.array([[np.cos(theta),-np.sin(theta)],
                    [np.sin(theta), np.cos(theta)]])
        R = R * r

        T = np.transpose(o_x)@v
        dot = np.linalg.norm(o_x)*np.linalg.norm(v)
        if dot == 0:
            phi = 0
        else:
            phi = np.arccos(T/dot)
        u_COLLAVOID[i] = (gamma*R@v*phi*np.exp(-beta*phi))
    
    u = np.sum(u_COLLAVOID, axis=0)
    
    return u
plt.figure(1, figsize=(6, 6))
#(plot_goal,) = plt.plot(u_COLLAVOID[0],u_COLLAVOID[1], "gx", mew=3)
for obs in OBS:
    (plot_obs,) = plt.plot(OBS[0], OBS[1], "rx", mew= 5)
    #(plot_path,) = plt.plot(u_3 [:, 0], u_3 [:, 1], "b", lw=2)
    plt.title("DMP system - obstacle avoidance")
    plt.axis("equal")
    plt.xlim([-1.1, 1.1])
    plt.ylim([-1.1, 1.1])
    plt.show()


def main(with_obs=False):
    env = Env(with_obs)  # env = Env(with_obs=True)
    env.simulate()
    


if __name__ == "__main__":
    # main()
    main(with_obs = True)
    
