# Render loop

Trying to use a generic event loop instead of a blocking render loop:

https://vtk.org/doc/nightly/html/classvtkGenericRenderWindowInteractor.html

https://gitlab.kitware.com/vtk/vtk/-/blob/v8.2.0/Rendering/OpenGL2/vtkXRenderWindowInteractor.cxx#L249

About process events (vtk above 8)

https://discourse.vtk.org/t/custom-event-loop-for-generic-render-window-interactor/7415