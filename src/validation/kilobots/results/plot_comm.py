import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib import colorbar
from matplotlib.backends.backend_pdf import PdfPages
from mpl_toolkits.axes_grid1 import make_axes_locatable

num_robots = 25
shape_robots = (5,5)

# Load data from reception experiments
avg_tx_rate = 0
for run in range(5):
    filename = "rece_tau000_dist%03d.dat" % run
    data = np.loadtxt(filename);
    data = data[:,1:]
    ## np.set_printoptions(precision=4,linewidth=1000)
    ## print np.diagonal(data)
    for i in range(num_robots):
        if i%5 == 0:
            avg_tx_rate += data[i,i]
avg_tx_rate /= 25
## print "Average transmission rate", avg_tx_rate

## Load data from communication experiments
num_runs=10
msg_tx_avg = np.zeros((1,num_robots))
msg_rx_norm_avg = np.zeros((num_robots,num_robots))
for run in range(num_runs):
    filename = "comm_tau000_run%03d.dat" % run
    data = np.loadtxt(filename);
    data = data[:,1:]
    msg_tx = np.diagonal(data)
    msg_tx_avg += msg_tx
    msg_rx_norm = data / msg_tx[None,:] 
    msg_rx_norm_avg += msg_rx_norm
msg_tx_avg /= num_runs
msg_rx_norm_avg /= num_runs

msg_tx_prob = msg_tx_avg/avg_tx_rate
np.set_printoptions(precision=4,linewidth=1000)
print np.mean(msg_tx_prob), "+-", np.std(msg_tx_prob),


dist = np.zeros((num_robots,num_robots))
for i in range(num_robots):
    xi = i % shape_robots[0]
    yi = i / shape_robots[1]
    for j in range(num_robots):
        xj = j % shape_robots[0]
        yj = j / shape_robots[1]
        dist[i,j] = np.sqrt( (xi-xj)*(xi-xj) + (yi-yj)*(yi-yj) )

idx = 24

with PdfPages('comm_data.pdf') as pdf:
    f = plt.figure(figsize=(12,10))
    ax = plt.axes()
    im = plt.matshow(msg_tx_prob.reshape(shape_robots), vmin=0.98,vmax=1, cmap="bone")
    cb = plt.colorbar(shrink=0.8, aspect=10, ticks=[0.98,0.99,1.0])
    ## divider = make_axes_locatable(ax)
    ## cax = divider.append_axes("right", "10%", pad="3%")
    ## cb = colorbar.ColorbarBase(cax,cmap="bone")
    cb.set_label('message transmission probability',fontsize=20)
    cb.ax.tick_params(labelsize=12)
    pdf.savefig()
    plt.close()

    
    f = plt.figure(figsize=(24,20))
    ## f, axarr = plt.subplots(shape_robots[0], shape_robots[1], figsize=(20,20))
    for x in range(shape_robots[0]):
        for y in range(shape_robots[1]):
            idx = x*5+y
            adx = x*6+y+1
            ## ax = axarr[x,y]
            ax = plt.subplot(5, 6, adx)
            if x != 0:
                ax.get_xaxis().set_visible(False)
            if y != 0:
                ax.get_yaxis().set_visible(False)
                
            im = ax.matshow(msg_rx_norm[idx,:].reshape(shape_robots),  vmin=0, vmax=1, cmap="bone")
            ax.text(y,x,r'$\times$',va="center", ha="center",fontsize=30)
            
    ax = plt.subplot(1,6,6)
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("left", "20%", pad="3%")
    cb = colorbar.ColorbarBase(cax,ticks=[0,0.25,0.5,0.75,1], cmap="bone")
    cb.set_label('average message reception probability',fontsize=40)
    cb.ax.tick_params(labelsize=24)
    ax.set_axis_off()
    
    plt.tight_layout()
    pdf.savefig()
    plt.close()
    
    f = plt.figure(figsize=(24,20))
    for x in range(shape_robots[0]):
        for y in range(shape_robots[1]):
            idx = x*5+y
            adx = x*6+y+1
            ## ax = axarr[x,y]
            ax = plt.subplot(5, 6, adx)
            ax.set_xlim(0,6)
            ax.set_ylim(0,1)
            ax.set_xlabel("distance");
            ax.set_ylabel("reception probability");
            if x != 4:
                ax.get_xaxis().set_visible(False)
            if y != 0:
                ax.get_yaxis().set_visible(False)

            dists = dict()
            counts = dict()
            for r in range(25):
                dists[dist[idx,r]] = 0
                counts[dist[idx,r]] = 0
            for r in range(25):
                dists[dist[idx,r]] += msg_rx_norm[idx,r]
                counts[dist[idx,r]] += 1
            for k,v in dists.items():
                dists[k] = v/counts[k]

            dsort = sorted(dists.keys())
            dvals = [dists[k] for k in dsort] 
            ax.plot(dsort, dvals, color='darkgreen', marker='o', linestyle='solid', linewidth=3, markersize=12)
    #        ax.plot(dist[idx,:], msg_rx_norm[idx,:],".")
    plt.tight_layout()
    pdf.savefig()
    plt.close()

