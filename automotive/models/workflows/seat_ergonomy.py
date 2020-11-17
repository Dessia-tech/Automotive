from dessia_common import workflow
import automotive.interior as interior

cockpit = workflow.TypedVariable(interior.Cockpit, name='Cockpit')

population_instanciator = workflow.ClassMethod(interior.Population, 'random_population')


mark_cockpit = workflow.ModelMethod(interior.Population, 'mark_cockpit')
extremas = workflow.ModelMethod(interior.SeatErgonomyAnalysis, 'extremas')
interior_instanciator = workflow.InstanciateModel(interior.CarInterior)#cockpit: 0

cockpit_pipe1 = workflow.Pipe(cockpit, mark_cockpit.inputs[1])
cockpit_pipe2 = workflow.Pipe(cockpit, interior_instanciator.inputs[0])
population_pipe = workflow.Pipe(population_instanciator.outputs[0],
                                mark_cockpit.inputs[0])
marks_pipe = workflow.Pipe(mark_cockpit.outputs[0],
                           extremas.inputs[0])

passenger_pipe = workflow.Pipe(extremas.outputs[0],
                               interior_instanciator.inputs[4])

simple_analysis = workflow.Workflow([population_instanciator,
                                            mark_cockpit],
                                           [population_pipe],
                                            mark_cockpit.outputs[0],
                                            name='Simple analysis'
                                           )


seat_analysis_workflow = workflow.Workflow([population_instanciator,
                                            mark_cockpit,
                                            interior_instanciator,
                                            extremas],
                                           [population_pipe, marks_pipe,
                                            passenger_pipe,
                                            cockpit_pipe1,
                                            cockpit_pipe2],
                                            interior_instanciator.outputs[0],

                                           )
