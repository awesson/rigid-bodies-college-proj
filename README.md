# This is an old college project that used to use svn. I don't remember the state it was in, but I want to save it on github for prosterity.


This is an amortized version of a rigid body simulation.

make backend will compile the backend executible. The backend is run with,
./backend port <number> and optionally [loop time]
This will run the ridig body simulation and send data to any frontend clients
who connect to the given port. If loop time is given then the simulation will
reset after that many seconds. The esc or 'q' button will end the backend simulation.

To connect to the backend remotely, compile the frontend binary with
'make frontend' and run with,
./frontend [hostname] [portnumber]
This will read data from the backend and update a local copy of the rigid
bodies. The rigid bodies are rendered on the frontend and the camera can be
controlled with the mouse buttons. Left click-drag will rotate the view, right
click-drag will zoom in and out, and middle button click-drag will translate 
the center of view. To close a frontend view click esc or 'q'.
