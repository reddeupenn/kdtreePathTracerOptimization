# kdtreePathTracerOptimization
KD tree optimization for path tracing

CIS565 Final Project Fall 2016 
Rony Edde

### K-D Tree optimizations for path tracer
Shooting rays through a complex scene with a large number of
polygons can be heavy and exponentially slow even on the GPU.  Axis
Aligned Bounding boxes help improve performance, however this still
doesn't solve the problem when rays intersect a complex object with
millions of triangles.

Using a K-D Tree to represent the triangles reduces the overall
complexity of the lookup and the time needed to find the
intersecting triangles.  There is a cost of computation for
generating the KD-Tree but in most cases the scene is mostly static
geometry which should benefit from this data structure.

* Path tracing hurdles.
Complex geometry tends to take a lot of memory.  The current path
tracer is unable to load complex geometry and crashes when the limit
is reached.  GPUs don't have enough memory to compete with CPUs 
and a typical path tracer loading millions of triangles can easily crash
the graphics card.  Just accessing the entire geometry without an optimized data structure can be close to impossible without resorting to optimizations.  However, even after optimizations, this can remain a major issue with high risks of overwhelming the GPU memory and resulting in a crash.  To solve this, we must resort to streaming.  If the object loaded has 1 million triangles, it would require 192MB of GPU memory on a 64 bit machine.  In today's applications and games 1 million polygons is considered low resolution.  We can see how this quickly becomes an issue.

Streaming alone, however presents additional complexities for optimization.  KD-Trees will need to be adaptive and fast in order to account for this.  Another option is to split the complexity in sub-trees in order to fit in memory, thus reducing the impact caused by streaming. This could have an impact on the benefit of using KD-Trees and can also present issues when rendering double sided geometry.

* Initial tentative road map
The project will consist of coming up with and implementing a K-D Tree that solves these issues with CPU and GPU generation.

