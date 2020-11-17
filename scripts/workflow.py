from automotive.models.workflows import seat_analysis_workflow
from automotive.models import cockpit, back_seat

import volmdlr as vm
# seat_analysis_workflow.plot_jointjs()

seat_analysis = seat_analysis_workflow.run({0:1000,
                              2:vm.Point3D(0, 0.3, 0.),
                              3: back_seat,
                              4: vm.Point2D(0, 0.),
                              6:cockpit})

seat_analysis.output_value.babylonjs()