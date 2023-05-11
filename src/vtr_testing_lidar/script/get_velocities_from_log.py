def vel_from_split(split):
    return [float(x) for x in split]

with open('/home/krb/ASRL/temp/radar/boreas/boreas-objects-v1/boreas-objects-v1/vtr-2023-04-06_21-11-15_293485029Z.log') as f:
    lines = f.readlines()

#gt_found (next row:)
#w_m_r_in_r

vels = []

for idx, line in enumerate(lines):
    if 'gt_found' in line:
        vels.append(vel_from_split(lines[idx + 1].split()[6:]))
    if 'w_m_r_in_r is:' in line:
        vels.append(vel_from_split(lines[idx].split()[7:]))

# vels = vels[:10876]
print(len(vels))

with open('/home/krb/ASRL/temp/radar/boreas/boreas-objects-v1/boreas-objects-v1/vtr-2023-04-06_21-11-15_293485029Z-velocities.txt', 'w') as f:
    for v in vels:
        f.write('{} {} {} {} {} {}\n'.format(v[0], v[1], v[2], v[3], v[4], v[5]))
