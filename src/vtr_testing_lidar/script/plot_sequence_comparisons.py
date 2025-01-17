from pyboreas.utils.utils import get_transform, get_inverse_tf
import matplotlib.pyplot as plt
import utm
import numpy as np
import folium

# coords is a list of (lat, lng) tuples
def plot_polyline_folium(coords, coords2=None, graph_map=None, tiles="openstreetmap", zoom=1, fit_bounds=True, color='#FF0000',
                         color2='#0000FF', linewidth=4, opacity=1, popup=None, attr=None, **kwargs):
    """
    Plot a list of (lat, lng) tuples on an interactive folium web map.
    ----------
    coords : List of (lat, lng) tuples
    graph_map : folium.folium.Map or folium.FeatureGroup
        if not None, plot the graph on this preexisting folium map object
    popup : string
        edge attribute to display in a pop-up when an edge is clicked
    tiles : string
        name of a folium tileset
    zoom : int
        initial zoom level for the map
    fit_bounds : bool
        if True, fit the map to the boundaries of the route's edges
    color : string
        color of the lines
    linewidth : numeric
        width of the lines
    opacity : numeric
        opacity of the lines
    kwargs : dict
        Extra keyword arguments passed through to folium
    Returns
    -------
    graph_map : folium.folium.Map
    """
    lat_sum = 0
    lng_sum = 0
    lats = []
    lngs = []
    for lat, lng in coords:
        lat_sum += lat
        lng_sum += lng
        lats.append(lat)
        lngs.append(lng)
    if coords2 is not None:
        for lat, lng in coords2:
            lat_sum += lat
            lng_sum += lng
            lats.append(lat)
            lngs.append(lng)
    if coords2 is None:
        centroid = (lat_sum / len(coords), lng_sum / len(coords))
    else:
        centroid = (lat_sum / (len(coords) + len(coords2)), lng_sum / (len(coords) + len(coords2)))

    if graph_map is None:
        graph_map = folium.Map(location=centroid, zoom_start=zoom, tiles=tiles, attr=attr)

    pl = folium.PolyLine(locations=coords, popup=popup, color=color, weight=linewidth, opacity=opacity, **kwargs)
    pl.add_to(graph_map)
    if coords2 is not None:
        pl2 = folium.PolyLine(locations=coords2, popup=popup, color=color2, weight=linewidth, opacity=opacity, **kwargs)
        pl2.add_to(graph_map)

    if fit_bounds and isinstance(graph_map, folium.Map):
        bounds = [(min(lats), min(lngs)), (max(lats), max(lngs))]
        graph_map.fit_bounds(bounds)

    return graph_map

mapbox_token = 'pk.eyJ1Ijoia2VlbmJ1cm4yMDA0IiwiYSI6ImNraHh1bm13dTA1cXEycG4wbTdvZ2xlY3YifQ.dtUB8qJaN09IjcfcmnOc1Q'
use_satellite = False
# graph_map = None

old_path = "/workspace/nas/ASRL/2021-Boreas/boreas-objects-v1/applanix_20230405backup//lidar_poses.csv"
new_path = "/home/krb/ASRL/boreas-objects-poses/lidar_poses.csv"

fold = open(old_path, 'r')
fnew = open(new_path, 'r')
header = fold.readline()  # kick out the header
header = fnew.readline()  # kick out the header
oldlines = fold.readlines()
newlines = fnew.readlines()
fold.close()
fnew.close()

assert len(oldlines) == len(newlines)

seq_indices = [0]
t_prev = int(oldlines[0].split(',')[0])
seq_times = [t_prev]
gt = [float(x) for x in oldlines[0].split(',')]
T_l0_enu = get_inverse_tf(get_transform(gt))

Told = []
Tnew = []
T_enu_old = []
T_enu_new = []

new_point_threshold = 1.0

for idx, line in enumerate(oldlines):
    t = int(line.split(',')[0])
    if t - t_prev > 5e5:
        print(seq_indices[-1])
        gt = [float(x) for x in line.split(',')]
        T_l0_enu = get_inverse_tf(get_transform(gt))
        fig, axs = plt.subplots()
        axs.plot([T[0, 3] for T in Told], [T[1, 3] for T in Told], 'r', label='old')
        axs.plot([T[0, 3] for T in Tnew], [T[1, 3] for T in Tnew], 'b', label='new')
        axs.legend()
        axs.axis('equal')

        plt.savefig('/home/krb/ASRL/boreas-objects-poses/{}-{}.pdf'.format(seq_indices[-1], seq_times[-1]))
        plt.close()

        # create folium plot
        coords0 = []
        coords1 = []

        x0_prev = 0
        y0_prev = 0
        x1_prev = 0
        y1_prev = 0

        for T0, T1 in zip(T_enu_old, T_enu_new):
            x0 = T0[0, 3]
            y0 = T0[1, 3]
            x1 = T1[0, 3]
            y1 = T1[1, 3]
            if np.sqrt((x0 - x0_prev)**2 + (y0 - y0_prev)**2) > new_point_threshold:
                lat0, lng0 = utm.to_latlon(x0, y0, 17, 'N')
                coords0.append((lat0, lng0))
                x0_prev = x0
                y0_prev = y0
            if np.sqrt((x1 - x1_prev)**2 + (y1 - y1_prev)**2) > new_point_threshold:
                lat1, lng1 = utm.to_latlon(x1, y1, 17, 'N')
                coords1.append((lat1, lng1))
                x1_prev = x1
                y1_prev = y1

        map = None
        if use_satellite:
            tiles = 'https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.png?access_token=' + str(mapbox_token)
            map = plot_polyline_folium(coords0, coords2=coords1, graph_map=map, tiles=tiles, attr='Mapbox')
        else:
            map = plot_polyline_folium(coords0, coords2=coords1, graph_map=map)
        
        map.save('/home/krb/ASRL/boreas-objects-poses/{}-{}.html'.format(seq_indices[-1], seq_times[-1]))
        
        Told.clear()
        Tnew.clear()
        T_enu_new.clear()
        T_enu_old.clear()
        seq_indices.append(idx)
        seq_times.append(t)
    
    gt_old = [float(x) for x in line.split(',')]
    gt_new = [float(x) for x in newlines[idx].split(',')]

    T_enu_lk_old = get_transform(gt_old)
    T_enu_lk_new = get_transform(gt_new)

    T_enu_old.append(T_enu_lk_old)
    T_enu_new.append(T_enu_lk_new)

    T_l0_lk_old = T_l0_enu @ T_enu_lk_old
    T_l0_lk_new = T_l0_enu @ T_enu_lk_new

    Told.append(T_l0_lk_old)
    Tnew.append(T_l0_lk_new)

    t_prev = t
    
# graph_map.save('/home/krb/ASRL/boreas-objects-poses/poses.html')