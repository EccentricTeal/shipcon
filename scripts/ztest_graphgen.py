#!/usr/bin/env python

import numpy
import matplotlib.pyplot
import sys

#Graph
argv = sys.argv
filename = argv[1]
outname = filename.replace('graph_', '')
outname = outname.replace('.csv', '')

fig = matplotlib.pyplot.figure()
sub1 = fig.add_subplot(1,1,1)

graph_data = numpy.loadtxt(filename, delimiter=',')
graph_time = graph_data[:,0]
graph_yaw = graph_data[:,1]
graph_delta = graph_data[:,2]
graph_pwr = graph_data[:,3]
graph_hdg = graph_data[:,4]

sub1.plot(graph_time, graph_delta, color='black', linestyle='solid', label=r'$\delta - t$')
sub2 = sub1.twinx()
sub2.plot(graph_time, graph_yaw, color='red', linestyle='solid', label=r'$\phi - t$')
[h1, l1] = sub1.get_legend_handles_labels()
[h2 ,l2] = sub2.get_legend_handles_labels()


sub1.set_xlabel(r'$t$  [sec]')
sub1.set_ylabel(r'$\delta$  [deg]')
sub1.set_ylim([-50.0, 50.0])
sub2.set_ylabel(r'$\phi$  [deg]')
sub2.set_ylim([-50.0, 50.0])

sub1.grid(True)
sub1.legend(h1+h2, l1+l2, loc='lower right')

fig.savefig(outname + '.eps')
