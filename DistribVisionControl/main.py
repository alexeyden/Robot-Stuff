import sys

from mockup import world, visual, server

objects = [
    [(3, 7), (2.5, 6), (5, 5), (7, 7.5), (5.5, 8)],
    [(5.8, 0.3), (4.7, 0.8), (5.5, 1.6), (7.3, 1.5), (7.1, 0.6)],
    [(9.1, 3.6), (8.6, 4.3), (9.2, 5.0), (9.8, 4.5), (9.8, 3.6)],
    [(11.9, 8.0), (10.8, 8.3), (10.7, 9.1), (11.8, 9.1), (12.2, 8.8)],
    [(16.1, 3.0), (18.8, 4), (17, 7), (15, 6.4), (14.6, 4.2)],
    [(3, 13.2), (2.5, 11), (5, 12.2), (7, 12.1), (5.5, 13)]
]

w = world.World()

for i, o in enumerate(objects):
    new_obj = world.WorldObject(o, str(i + 1))
    w.objects.append(new_obj)

renderer = visual.WorldRenderer(w)
srv = server.Server(8080, renderer.server_pipe)
srv.run()

renderer.run()

srv.stop()
