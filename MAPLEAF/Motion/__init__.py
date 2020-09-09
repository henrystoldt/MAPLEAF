'''
Generalized Rigid body motion integration functionality.
Main class (6Dof) is `MAPLEAF.Motion.RigidBody.RigidBody`.  
Fundamental data types used throughout the simulator defined in:

* `Vector`
* `Quaternion` - represents orientation
* `AngularVelocity` 
* `Inertia` - stores component masses and moments of inertias
* `ForceMomentSystem`  - stores a repositionable force-moment system

3Dof and 6DoF Rigid body states are composed of these fundamental data types and defined in `RigidBodyStates`

Generalized constant and adaptive time stepping integrators are defined in `Integration`

.. image:: https://www.researchgate.net/profile/Alireza_Abbaspour2/publication/326452421/figure/fig1/AS:701040399753217@1544152464477/The-aircraft-coordinate-system-34.jpg
'''