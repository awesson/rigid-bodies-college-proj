# $Id: gfx-config.in 343 2008-09-13 18:34:59Z garland $

CXX = g++
CXXFLAGS = -O3 -Wall -Wno-sign-compare -Iinclude -DHAVE_CONFIG_H 
OBJS = csapp.o imageio.o imageio_v2.o System.o integrator.o quaternion.o matrix.o Math.o Color.o Material.o Box.o Body.o rts.o

backend: backend.o $(OBJS) BoxMesh.o
	$(CXX) -o $@ $^ -lpng -lpthread -framework GLUT -framework OpenGL
frontend: frontend.o $(OBJS) BoxMesh_front.o
	$(CXX) -o $@ $^ -lpng -lpthread -framework GLUT -framework OpenGL
local: LocalRigidBodies.o $(OBJS) BoxMesh.o
	$(CXX) -o $@ $^ -lpng -lpthread -framework GLUT -framework OpenGL
clean:
	rm frontend.o backend.o LocalRigidBodies.o BoxMesh.o BoxMesh_front.o $(OBJS) frontend backend local
