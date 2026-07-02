# meshes/

Drop converted **STL / DAE / OBJ** meshes here (not `.step`/`.sldprt` — URDF
can't read those). Convert from your CAD first (SolidWorks/FreeCAD → STL),
mind the mm→m scale, and decimate heavy meshes.

To use a mesh, replace a primitive `<geometry>` in the xacro, e.g.:

```xml
<geometry>
  <mesh filename="package://helios_description/meshes/chassis.stl"
        scale="0.001 0.001 0.001"/>   <!-- if exported in millimeters -->
</geometry>
```

Keep `<collision>` as simple primitives even when the visual is a mesh.
