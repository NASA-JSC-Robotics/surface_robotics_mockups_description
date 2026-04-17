# Notes on parameter selection for flexcomp body (cpc handle)

## Disclaimer

All of these parameters came from trial and error.  
As I learned more about how the flexcomps work, I was able to get the behavior I wanted, but the way I did this is not necessarily the only way -- for instance, I suspect there is a reasonable way to use a mesh type instead of a grid type and there is likely a reasonable way to model this as more explicitly 3 dimensional.

## *Flexcomp Attributes*

### *type*

A two dimensional grid was selected because everythig else failed.  
I tried using a mesh file from a .obj file, but getting the faces and verticies right to get it to bend the way I wanted it to led me to basically recreate the grid structure in the obj. 
It was easier to iterate and test using the *grid* type than to edit the mesh file everytime I wanted to try something different.

### *dim*

At first I thought this shold be 3 dimensional, but there is no "flex" in the third dimension, so it wasn't necessary. 
From a collision standpoint the *radius" parameter gives the strap volume.  
Using the third dimension resulted in unnecessarily complex collision computation and made the strap stretch in a weird way. 
*Stretching may or may not be related to the existance of the third dimension and more to do with how I modeled it*

### *count*, and *spacing*

Count and spacing determine the size of the object.  
Count determines how many verticies will be in each dimension. Count*Spacing gives the length in each dimension.  
More verticies means more flexibilty, since it only bends on a an "edge".  
If we want the strap to flex in the short dimension, we need to make count 3 (and shrink spacing) so that there is an edge down the center of the strap to bend around. 
I preferred the motion without the strap bending. 
The spacing on the last dimension doesn't matter as long as it is larger than the radius. 
I have it set to "1".

### *radius*

The radius is, in this case, is essentially the thickness of the strap from a visual and collision standpoint.  
I set it to 0.002 for a 4mm thick strap.

### *rgba*

Selected from the CPC baseball card.  
We should set it to whatever we want.

### *edge*

### *equality*

Set to "true" so that the strap maintains it's basic shape.  
The documentation here is not clear (at least to me), but trial and error says it has to be "true".

### *solref* 
Not set with much thought.

## *contact*

### *condim*

Tried "3" (Regular frictional contact, opposing slip in the tangent plane.) and "4" (Frictional contact, opposing slip in the tangent plane and rotation around the contact normal. 
This is useful for modeling soft contacts (independent of contact penetration).)  
Didn't see a big difference either way.

### *solref*
See above

## *elasticity*

### *young*

I wanted no stretch, so I set the young's constant high.

### *poisson*

Measures the narrowing (transverse strain) compared to stretching (axial strain).  
I looked up the number for polyester webbing. 
Poisson constants were 0.30 - 0.39.

### *thickness*

"Shell thickness, units of length; only for used 2D flexes. 
Used to scale the stretching stiffness. This thickness can be set equal to 2 times the radius in order to match the geometry, but is exposed separately since the radius might be constrained by considerations related to collision detection." (https://mujoco.readthedocs.io/en/latest/XMLreference.html#flex-elasticity)

I set it to higher than 2x radius, since I wanted a stiffer, inelastic strap.
