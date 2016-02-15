#include "System.h"
#include <algorithm>

#define LEVEL_ITER 5
// globals for tarjan's algorithm
std::vector<Body*> top_sorted;
std::stack<Body*> S;
int SCC_num;

static double *curr_pos, *curr_vel, *prev_pos, *prev_vel;

System::System(std::vector<Body*> &i_bVector) : bVector(i_bVector),
                                               size(bVector.size())
{
	curr_pos = new double[size_pos()];
	curr_vel = new double[size_vel()];
	prev_pos = new double[size_pos()];
	prev_vel = new double[size_vel()];
}

System::~System(void)
{
	for(int i = 0; i < size; ++i){
		delete bVector[i];
	}
    bVector.clear();
	delete[] curr_pos;
	delete[] curr_vel;
	delete[] prev_pos;
	delete[] prev_vel;
}

/**
 * zeros out the forces and torques
 **/
void System::zero_forces()
{
    for(int i = 0; i < bVector.size(); ++i){
        bVector[i]->forces = Vec3(0, 0, 0);
        bVector[i]->torques = Vec3(0, 0, 0);
    }
}

/**
 * adds gravity to the forces
 **/
void System::add_gravity()
{
    for(int i = 0; i < bVector.size(); ++i){
        if(bVector[i]->inv_mass > EPSILON) // dont add gravity to static objects
            bVector[i]->forces = Vec3(0, -g / bVector[i]->inv_mass, 0);
    }
}

/**
 * calculates impulse forces and torques for collision detection
 **/
bool System::collsion_detect(const RBIntegrator* pIntegrator, double dt, double* prev_pos, double* prev_vel)
{
	Vec3 p, p1, p2, normal, r1, r2;
	Body *b1, *b2;
	bool has_collisions = false;
	
    for(int i = 0; i < bVector.size(); ++i){
		for(int k = i+1; k < bVector.size(); ++k){
			b1 = bVector[i];
			b2 = bVector[k];
#if USE_XENOCOLLIDE
			if(Body::intersection_test(b1, b2, p1, p2, normal))
#else
			if(b1->intersection_test(b2, p, normal))
#endif
			{
#if USE_XENOCOLLIDE	
				// get the relative position of the collision points in the x', v' frame
				r1 = p1 - b1->Position;
				r2 = p2 - b2->Position;
				// The intersection test returns a normal relative to b2,
				// but the collision resolution uses a normal relative to b1.
				normal = -normal;
#else
				// get the relative position of the collision points in the x', v' frame
				r1 = p - b1->Position;
				r2 = p - b2->Position;
#endif
				
				// set the system back to the x', v state to apply collision forces
				get_state_vel(curr_vel + i*VEL_STATE_SIZE, i);
				get_state_vel(curr_vel + k*VEL_STATE_SIZE, k);
				set_state_vel(prev_vel + i*VEL_STATE_SIZE, i);
				set_state_vel(prev_vel + k*VEL_STATE_SIZE, k);
				
				if(resolve_collisions(b1, b2, r1, r2, normal, -1, false))
				{
					has_collisions = true;
					
					// Save off the new v state
					get_state_vel(prev_vel + i*VEL_STATE_SIZE, i);
					get_state_vel(prev_vel + k*VEL_STATE_SIZE, k);
					
					// Update the x' for the bodies in this collision
					set_state_pos(prev_pos + i*POS_STATE_SIZE, i);
					set_state_pos(prev_pos + k*POS_STATE_SIZE, k);
					pIntegrator->integrate_vel(*this, dt, i);
					pIntegrator->integrate_vel(*this, dt, k);
					pIntegrator->integrate_pos(*this, dt, i);
					pIntegrator->integrate_pos(*this, dt, k);
				}
				else
				{
					// Save off the new v state
					get_state_vel(prev_vel + i*VEL_STATE_SIZE, i);
					get_state_vel(prev_vel + k*VEL_STATE_SIZE, k);
				}
				
                // reset the system to x', v' for the rest of the collisions to be resolved
				set_state_vel(curr_vel + i*VEL_STATE_SIZE, i);
				set_state_vel(curr_vel + k*VEL_STATE_SIZE, k);
			}
        }
    }

	return has_collisions;
}

/**
 * calculates impulse forces and torques for contact detection
 **/
bool System::contact_detect(const RBIntegrator* pIntegrator, double dt, double* prev_pos, int iter, bool is_shock_prop)
{
	Vec3 r1, r2, p, p1, p2, normal;
	Body *b1, *b2;
	bool has_contacts = false;
	bool had_contact_this_iter = false;
	int count = 0, cur_SCC = 0, SCC_head_body = 0;
	for(int i = 0; i < size || count < LEVEL_ITER; ++i){
		if(i == size || bVector[i]->SCC_num != cur_SCC)
		{ // Reached the last body in the current strongly connected component
			count++;
			
			if(count == LEVEL_ITER || (count != 0 && !had_contact_this_iter))
			{ // Move onto the next strongly connected component if this is the max number of iterations per level
			  // or if we already did one iteration and found no contacts.

				if(i == size)
				{ // This was the last SCC so just end the loop
					break;
				}

				// set the bodies in this level to be static before moving
				// on to the next level if applying shock propagation
				if(is_shock_prop)
				{
					for(int k = SCC_head_body; k < i; ++k)
					{
						Body *b = bVector[k];
						b->inv_mass = 0;
						b->Iinv = Matrix3(Vec3(0,0,0), Vec3(0,0,0), Vec3(0,0,0));
					}
				}

				cur_SCC++;
				SCC_head_body = i;
				count = 0;
			}
			else if(count < LEVEL_ITER)
			{ // Move on to the next iteration through this strongly connected component
				i = SCC_head_body;
			}
			
			had_contact_this_iter = false;
		}
		
		b1 = bVector[i];
		for(int k = i - 1; k >= 0; --k){
			b2 = bVector[k];

#if USE_XENOCOLLIDE
			if(Body::intersection_test(b1, b2, p1, p2, normal))
#else
			if(b1->intersection_test(b2, p, normal))
#endif
			{
#if USE_XENOCOLLIDE	
				// get the relative position of the collision points in the x', v' frame (TODO: make this in the x, v' frame)
				r1 = p1 - b1->Position;
				r2 = p2 - b2->Position;
				// The intersection test returns a normal relative to b2,
				// but the collision resolution uses a normal relative to b1.
				normal = -normal;
#else
				// get the relative position of the collision points in the x', v' frame
				r1 = p - b1->Position;
				r2 = p - b2->Position;
#endif

				had_contact_this_iter = resolve_collisions(b1, b2, r1, r2, normal, iter, true) || had_contact_this_iter;
				has_contacts = had_contact_this_iter || has_contacts;
				
				if(had_contact_this_iter)
				{
					// Update the x' for the bodies in this collision
					set_state_pos(prev_pos + i*POS_STATE_SIZE, i);
					set_state_pos(prev_pos + k*POS_STATE_SIZE, k);
					pIntegrator->integrate_pos(*this, dt, i);
					pIntegrator->integrate_pos(*this, dt, k);
				}
			}
		}
	}
	
	// reset the masses and synch the momentum with
	// the velocity if shock propagation was used
	if(is_shock_prop)
	{
		for(int i = 0; i < size; ++i)
		{
			Body* b = bVector[i];
			b->inv_mass = b->construct_inv_mass;
			b->Iinv = b->R * b->Iinv_body * b->R_t;
			if(!IsZero(b->inv_mass))
			{
				b->Momentum = b->Velocity / b->inv_mass;
				Matrix3 I;
				inverse(&I, b->Iinv);
				b->AngularMomentum = I * b->Omega;
			}
		}
	}
	
	return has_contacts;
}

/**
 * Searches through each pair of bodies for intersection for the current state of the system, x and v'.
 * For each pair that is intersecting, the bodies are reset to the prev state, x and v.
 * Impulse forces are calculated and applied to the velocities of the bodies in the x and v state.
 * These new velocities with the collision information are stored back into the prev arrays.
 * If the is_contact flag is set then the collisions are precessed with a coefficient of restitution of 0
 * otherwise the minimum of the two restitutions are chosen and similarly for friction regardless of whether
 * it is collision of contact resolution.
 **/
bool System::resolve_collisions(Body *b1, Body *b2, Vec3 r1, Vec3 r2, Vec3 normal, int iter, bool is_contact)
{	
	Matrix3 K = b1->get_K(r1) + b2->get_K(r2);
	Matrix3 K_inv;
	inverse(&K_inv, K);
	Vec3 u_rel = b2->get_vel(r2) - b1->get_vel(r1);
	
	// check if bodies are non-separating in the current timestep
	if(u_rel*normal >= 0.0){
		return false; // non-separating, no contact
	}
	
	// has_collisions = true;
	double restitution;
	if(is_contact)
	{
		if(iter > 4)
		{ // gradually reduce speed
			restitution = 0.0;
		}
		else
		{
			restitution = -.2*(4-iter);
		}
	}
	else
	{
		restitution = std::min(b1->restitution, b2->restitution);
	}

    double friction = std::min(b1->coef_friction, b2->coef_friction);

    // check if static friction should be used
    Vec3 j_static = K_inv*(-restitution*(u_rel*normal)*normal - u_rel);
    double j_static_dot_normal = j_static*normal;
	Vec3 j;

    if(norm(j_static - (j_static_dot_normal)*normal)
     	<= friction*(j_static_dot_normal))
	{
        // static friction is ok
		j = j_static;
    }
	else
	{ // use kinetic friction
        double u_rel_dot_normal = u_rel*normal;
        Vec3 t = u_rel - (u_rel_dot_normal)*normal;
        unitize(t);
        Vec3 normal_minus_friction_t = normal - friction*t;
        double j_n = -(restitution + 1)*(u_rel_dot_normal) /
                    (normal*K*(normal_minus_friction_t));
        j = (j_n*(normal_minus_friction_t));
    }

    b1->Momentum -= j;
	b1->Velocity -= j * b1->inv_mass;
    b2->Momentum += j;
	b2->Velocity += j * b2->inv_mass;
    b1->AngularMomentum += cross(r1, -j);
	b1->Omega += b1->Iinv * cross(r1, -j);
    b2->AngularMomentum += cross(r2, j);
	b2->Omega += b2->Iinv * cross(r2, j);
	return true;
}

/**
 * take derivative of position/orientation assuming forces and torques have been calculated already
 **/
void System::eval_deriv_pos(double xdot[]){
    for(int i = 0; i < bVector.size(); ++i)
        eval_deriv_pos(xdot + i*POS_STATE_SIZE, i);
}

/* take derivative of vel/ang vel assuming forces and torques have been calculated already */
 void System::eval_deriv_vel(double xdot[]){
     /* update velocity/angular velocity */
     for(int i = 0; i < bVector.size(); ++i)
        eval_deriv_vel(xdot + i*VEL_STATE_SIZE, i);
}

void System::get_state_pos(double x[]) const{
    for(int i = 0; i < bVector.size(); ++i)
        get_state_pos(x + i*POS_STATE_SIZE, i);
}

void System::get_state_vel(double x[]) const{
    for(int i = 0; i < bVector.size(); ++i)
        get_state_vel(x + i*VEL_STATE_SIZE, i);
}

void System::set_state_pos(const double x[]){
    for(int i = 0; i < bVector.size(); ++i)
        set_state_pos(x + i*POS_STATE_SIZE, i);
}

void System::set_state_vel(const double x[]){
    for(int i = 0; i < bVector.size(); ++i)
        set_state_vel(x + i*VEL_STATE_SIZE, i);
}

/* get/set/eval functions for single bodies */
void System::get_state_pos(double x[], int i) const{
    Body *b = bVector[i];

	get_state_pos(x, b);
}

void System::get_state_vel(double x[], int i) const{
    Body *b = bVector[i];

	get_state_vel(x, b);
}

void System::set_state_pos(const double x[], int i){
    Body *b = bVector[i];

	set_state_pos(x, b);
}

void System::set_state_vel(const double x[], int i){
    Body *b = bVector[i];

	set_state_vel(x, b);
}

void System::get_state_pos(double x[], Body *b) const{
    // pos
    for(int k = 0; k < 3; ++k)
        x[k] = b->Position[k];

    // orientation
    x[3] = b->Orientation.w;
    x[4] = b->Orientation.x;
    x[5] = b->Orientation.y;
    x[6] = b->Orientation.z;
}

void System::get_state_vel(double x[], Body *b) const{
    // momentum
    for(int k = 0; k < 3; ++k)
        x[k] = b->Momentum[k];

    // angular momentum
    for(int k = 0; k < 3; ++k)
        x[k + 3] = b->AngularMomentum[k];
}

void System::set_state_pos(const double x[], Body *b){
    // pos
    for(int k = 0; k < 3; ++k)
        b->Position[k] = x[k];

    // orientation
    b->Orientation.w = x[3];
    b->Orientation.x = x[4];
    b->Orientation.y = x[5];
    b->Orientation.z = x[6];

    // R and R transpose
    b->Orientation = normalize(b->Orientation);
    b->Orientation.to_matrix(&(b->R));
    transpose(&(b->R_t), b->R);

    // world inverse inertia tensor
    b->Iinv = b->R * b->Iinv_body * b->R_t;
}

void System::set_state_vel(const double x[], Body *b){
    // momentum and velocity
    for(int k = 0; k < 3; ++k){
        b->Momentum[k] = x[k];
        b->Velocity[k] = x[k] * b->inv_mass;
    }

    // angular momentum
    for(int k = 0; k < 3; ++k)
        b->AngularMomentum[k] = x[k + 3];

    // angular velocity
    b->Omega = b->Iinv * b->AngularMomentum;
}

void System::eval_deriv_pos( double xdot[], int i){
    Body* b = bVector[i];

    // dx/dt
    for(int k = 0; k < 3; ++k)
        xdot[k] = b->Velocity[k];

    // d(quat)/dt
    Quaternion q_dot = 0.5 * Quaternion(0.0, b->Omega[0], b->Omega[1],
                                        b->Omega[2]) * b->Orientation;
    xdot[3] = q_dot.w;
    xdot[4] = q_dot.x;
    xdot[5] = q_dot.y;
    xdot[6] = q_dot.z;
}

void System::eval_deriv_vel( double xdot[], int i ){
    Body* b = bVector[i];

     // dp/dt
     for(int k = 0; k < 3; ++k)
         xdot[k] = b->forces[k];

     // dL/dt
     for(int k = 0; k < 3; ++k)
         xdot[k + 3] = b->torques[k];
}

/**
 * Topologically sorts the objects based on the contact graph.
 * Uses Tarjan's algorithm to condense strongly connected components.
 **/
void System::topological_tarjan(){
    int index = 0;
	SCC_num = 0;
    for(int i = 0; i < size; ++i){
        if(bVector[i]->index < 0){
            strongconnect(bVector[i], index);
        }
    }
    
	// copy over the sorted list and reset values used in the function
    for(int i = 0; i < size; i++){
        bVector[i] = top_sorted[i];
		bVector[i]->index = -1;
		bVector[i]->lowlink = -1;
    }

	top_sorted.clear();
}

/**
 * Recursive function to find the SCC in the graph and add them to the new,
 * topologically sorted list of bodies.
 **/
void System::strongconnect(Body *vertex, int &index){
    vertex->index = index;
    vertex->lowlink = index;
    index++;
    S.push(vertex);
    vertex->in_stack = true;
    
	// compare index values with all children
    for(int i = 0; i < vertex->in_contact_list.size(); i++){
        Body* child_vertex = vertex->in_contact_list[i];
        if(child_vertex->index < 0){ // recurse on child if index is undef
            strongconnect(child_vertex, index);
            if(vertex->lowlink > child_vertex->lowlink)
                vertex->lowlink = child_vertex->lowlink;
        } else{ // otherwise update the lowlink if the vertex is reachable from the child
            if(child_vertex->in_stack && (vertex->lowlink > child_vertex->index))
                vertex->lowlink = child_vertex->index;
        }
    }
    
    // check if v is a root node of the SCC
    if(vertex->lowlink == vertex->index){
        Body *tmp_vertex;
        // pop vertices off the stack above the current vertex
        // as those are in a SCC and move them to the sorted list.
        while((tmp_vertex = S.top()) != vertex){
            S.pop();
			tmp_vertex->in_stack = false;
			tmp_vertex->SCC_num = SCC_num;
            top_sorted.push_back(tmp_vertex);
        }
        // put the current vertex in the SCC as well
        S.pop();
		tmp_vertex->in_stack = false;
		tmp_vertex->SCC_num = SCC_num;
        top_sorted.push_back(tmp_vertex);
		SCC_num++;
    }
}

/**
 * Saves the current state of the system in a list which will be sent to the client.
 **/
void System::saveOutputData(std::vector<BodyInfo> &bodyData){
    for(int i = 0; i < size; ++i){
        bVector[i]->getInfo(bodyData[i]);
    }
}

void System::get_bodies(std::vector<Body*> & o_bodyVector){
    o_bodyVector = bVector;
}

unsigned int System::num_bodies() const{
    return size;
}

unsigned int System::size_pos() const{
    return size*POS_STATE_SIZE;
}

unsigned int System::size_vel() const{
    return size*VEL_STATE_SIZE;
}

