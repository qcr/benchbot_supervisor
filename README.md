# BenchBot Supervisor

Repository holds the BenchBot supervisor, which facilitates the communication between BenchBot API & the robot / simulator.

Proper documentation may come later (read: "here be dragons"). Here's some random notes that I think people might find helpful if they have to do stuff in here:

- Commmunication "upwards" with BenchBot API is done by Flask server through GET RESTful API requests (I haven't any reason to use POST yet)
- Communication "downwards" with robot / simulator is done via ROS publishers / subscribers
- The supervisor has access to the ROS TF tree but does not pass any of it up to the BenchBot API... this needs to be done properly at some point
- `src/supervisor_callbacks.py` is where we implement custom actions that should happen in the communication between API & Robot / simulator (and vice versa). This is pretty poorly documented at the moment & implementations of things are relatively hacky / unstructured. The implemented callbacks should be somewhat informative, but this part of the pipeline definitely needs work down the track.

