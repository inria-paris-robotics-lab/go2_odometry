import numpy as np

import hppfcl
import pinocchio as pin
import simple

class Simulation:
    def __init__(self, model, geom_model, visual_model, q0, v0, args):
        self.model = model
        self.geom_model = geom_model
        self.visual_model = visual_model
        self.q0 = q0
        self.v0 = v0
        self.args = args

        self.data = self.model.createData()
        self.geom_data = self.geom_model.createData()

        for col_req in self.geom_data.collisionRequests:
            col_req: hppfcl.CollisionRequest
            col_req.security_margin = 0.0
            col_req.break_distance = 0.0
            col_req.gjk_tolerance = 1e-6
            col_req.epa_tolerance = 1e-6
            col_req.gjk_initial_guess = hppfcl.GJKInitialGuess.CachedGuess
            col_req.gjk_variant = hppfcl.GJKVariant.DefaultGJK

        for patch_req in self.geom_data.contactPatchRequests:
            patch_req.setPatchTolerance(args.patch_tolerance)

        # Simulation parameters
        self.simulator = simple.Simulator(model, self.data, geom_model, self.geom_data)
        self.simulator.contact_solver_info = pin.ProximalSettings(
            args.tol, args.tol_rel, args.mu_prox, args.maxit
        )
        self.simulator.warm_start_constraint_forces = args.warm_start
        self.simulator.measure_timings = True
        # Contact patch settings
        self.simulator.constraints_problem.setMaxNumberOfContactsPerCollisionPair(
            args.max_patch_size
        )
        # Baumgarte settings
        self.simulator.constraints_problem.Kp = args.Kp
        self.simulator.constraints_problem.Kd = args.Kd
        if args.admm_update_rule == "spectral":
            self.simulator.admm_constraint_solver_settings.admm_update_rule = (
                pin.ADMMUpdateRule.SPECTRAL
            )
        elif args.admm_update_rule == "linear":
            self.simulator.admm_constraint_solver_settings.admm_update_rule = (
                pin.ADMMUpdateRule.LINEAR
            )
        else:
            print(f"ERROR - no match for admm update rule {args.admm_update_rule}")
            exit(1)
        self.dt = args.dt
        self.q = q0.copy()
        self.v = v0.copy()
        self.fext = [pin.Force(np.zeros(6)) for _ in range(model.njoints)]
        fps = min([self.args.max_fps, 1.0 / self.dt])
        self.dt_vis = 1.0 / float(fps)
        self.simulator.reset()

        pass

    def execute(self, tau):
        if self.args.contact_solver == "ADMM":
            self.simulator.step(self.q, self.v, tau, self.fext, self.dt)
        else:
            self.simulator.stepPGS(self.q, self.v, tau, self.fext, self.dt)
        #print(self.simulator.getStepCPUTimes().user)
        self.q = self.simulator.qnew.copy()
        self.v = self.simulator.vnew.copy()

        #print("elapsed simu time " + str(step_end - step_start))
        #time_until_next_step = self.dt_vis - (time.time() - step_start)
        #if time_until_next_step > 0:
        #    time.sleep(time_until_next_step)
    
    def get_state(self):
        return self.q, self.v
    
    def view_state(self, q):
        self.vizer.display(q)


def setPhysicsProperties(
    geom_model: pin.GeometryModel, material: str, compliance: float
):
    for gobj in geom_model.geometryObjects:
        if material == "ice":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.ICE
        elif material == "plastic":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.PLASTIC
        elif material == "wood":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.WOOD
        elif material == "metal":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.METAL
        elif material == "concrete":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.CONCRETE

        # Compliance
        gobj.physicsMaterial.compliance = compliance


def removeBVHModelsIfAny(geom_model: pin.GeometryModel):
    for gobj in geom_model.geometryObjects:
        gobj: pin.GeometryObject
        bvh_types = [hppfcl.BV_OBBRSS, hppfcl.BV_OBB, hppfcl.BV_AABB]
        ntype = gobj.geometry.getNodeType()
        if ntype in bvh_types:
            gobj.geometry.buildConvexHull(True, "Qt")
            gobj.geometry = gobj.geometry.convex


def addFloor(geom_model: pin.GeometryModel, visual_model: pin.GeometryModel):
    # Collision object
    # floor_collision_shape = hppfcl.Box(10, 10, 2)
    # M = pin.SE3(np.eye(3), np.zeros(3))
    # M.translation = np.array([0.0, 0.0, -(1.99 / 2.0)])
    floor_collision_shape = hppfcl.Halfspace(0, 0, 1, 0)
    # floor_collision_shape = hppfcl.Plane(0, 0, 1, 0)
    # floor_collision_shape.setSweptSphereRadius(0.5)
    M = pin.SE3.Identity()
    floor_collision_object = pin.GeometryObject("floor", 0, 0, M, floor_collision_shape)
    geom_model.addGeometryObject(floor_collision_object)

    # Visual object
    floor_visual_shape = hppfcl.Box(10, 10, 0.01)
    floor_visual_object = pin.GeometryObject(
        "floor", 0, 0, pin.SE3.Identity(), floor_visual_shape
    )
    visual_model.addGeometryObject(floor_visual_object)
        
def addSystemCollisionPairs(model, geom_model, qref):
    """
    Add the right collision pairs of a model, given qref.
    qref is here as a `T-pose`. The function uses this pose to determine which objects are in collision
    in this ref pose. If objects are in collision, they are not added as collision pairs, as they are considered
    to always be in collision.
    """
    data = model.createData()
    geom_data = geom_model.createData()
    pin.updateGeometryPlacements(model, data, geom_model, geom_data, qref)
    geom_model.removeAllCollisionPairs()
    num_col_pairs = 0
    for i in range(len(geom_model.geometryObjects)):
        for j in range(i+1, len(geom_model.geometryObjects)):
            # Don't add collision pair if same object
            if i != j:
                gobj_i: pin.GeometryObject = geom_model.geometryObjects[i]
                gobj_j: pin.GeometryObject = geom_model.geometryObjects[j]
                if gobj_i.name == "floor" or gobj_j.name == "floor":
                    num_col_pairs += 1
                    col_pair = pin.CollisionPair(i, j)
                    geom_model.addCollisionPair(col_pair)
                else:
                    if gobj_i.parentJoint != gobj_j.parentJoint or gobj_i.parentJoint == 0:
                        if gobj_i.parentJoint != model.parents[gobj_j.parentJoint] and gobj_j.parentJoint != model.parents[gobj_i.parentJoint] or gobj_i.parentJoint == 0 or gobj_j.parentJoint == 0:
                            # Compute collision between the geometries. Only add the collision pair if there is no collision.
                            M1 = geom_data.oMg[i]
                            M2 = geom_data.oMg[j]
                            colreq = hppfcl.CollisionRequest()
                            colreq.security_margin = 1e-2 # 1cm of clearance
                            colres = hppfcl.CollisionResult()
                            hppfcl.collide(gobj_i.geometry, M1, gobj_j.geometry, M2, colreq, colres)
                            if not colres.isCollision():
                                num_col_pairs += 1
                                col_pair = pin.CollisionPair(i, j)
                                geom_model.addCollisionPair(col_pair)
    print("Num col pairs = ", num_col_pairs)
