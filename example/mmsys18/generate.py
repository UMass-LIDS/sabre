# Copyright (c) 2018, Kevin Spiteri
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import math
import os
import subprocess
import json
import sys
import threading

# TODO: Clean up to give an easier-to-understand example how to use Sabre!

def load_json(path):
    with open(path) as file:
        obj = json.load(file)
    return obj

def cdf(l, margin = 0.025):
    l = sorted(l)
    range = l[-1] - l[0]
    if range > 0:
        margin *= range
    inc = 1 / len(l)
    c = []
    y = 0
    if range == 0:
        c += [[l[0] - margin, y]]
    for x in l:
        c += [[x, y]]
        y += inc
        c += [[x, y]]
    if range == 0:
        c += [[l[-1] + margin, y]]
    return c

def mean_stddev(l):
    mean = sum(l) / len(l)
    var = sum([(x - mean) * (x - mean) for x in l]) / len(l)
    stddev = math.sqrt(var)
    return (mean, stddev)

def thread_run_sabre(results, command):
    completed = subprocess.run(command, stdout = subprocess.PIPE)
    for line in completed.stdout.decode('ascii').split('\n'):
        l = line.split(':')
        if len(l) != 2:
            continue
        if l[0] in results:
            results[l[0]].append(float(l[1]))

def thread_run_gnuplot(plotting):
    subprocess.run('gnuplot', input = plotting.encode('ascii'))

def do_figure(prefix, subfigs, algorithms, metrics, term = None):
    print(prefix + ' ', end = '')

    plotting_threads = []

    for subfig in subfigs:
        title = subfig[0]
        dir = subfig[1]
        args1 = subfig[2]

        print(title + ' ', end = '')

        # info['metric_name']['algorithm_name'] = (mean, stddev, 'name of .dat file')
        info = {m[0]: {} for m in metrics}
        
        plot_mark_offset = 0
        for algorithm in algorithms:
            plot_mark_offset += 1
            name = algorithm[0]
            args = args1 + algorithm[1]
        
            print(name + ' ', end = '')
        
            results = {m[1]: [] for m in metrics}
        
            cnt = 0
            max_threads = 5
            threads = []
            for trace in os.listdir(dir)[:]: # use this line to limit directory size
                cnt += 1
                print('%d' % cnt, end = '')
                sys.stdout.flush()
        
                if len(threads) >= max_threads:
                    for t in threads:
                        if not t.is_alive():
                            t.join()
                            threads.remove(t)
                            break
        
                if len(threads) >= max_threads:
                    threads[0].join()
                    threads.pop(0)
        
                command = ['python3', './sabre-mmsys18.py', '-n', dir + '/' + trace] + args
                t = threading.Thread(target = thread_run_sabre, args = (results, command))
                threads.append(t)
                t.start()
        
                print('\b' * len(str(cnt)), end = '')
                print(' '  * len(str(cnt)), end = '')
                print('\b' * len(str(cnt)), end = '')
            for t in threads:
                t.join()
        
            print('\b' * (len(name) + 1), end = '')
            print(' '  * (len(name) + 1), end = '')
            print('\b' * (len(name) + 1), end = '')
        
            for metric in metrics:
                config = metric[2] if len(metric) > 2 else {}
                samples = results[metric[1]]
                points = cdf(samples)
                median = (points[(len(points) - 1) // 2][0] + points[len(points) // 2][0]) / 2
                stats = (median, ) + mean_stddev(samples)
                datname = ('tmp/' + prefix + '-' +
                           title.replace(' ', '-') + '-' +
                           metric[0].replace(' ', '-') + '-' +
                           algorithm[0].replace(' ', '-') + '.dat')
                info[metric[0]][algorithm[0]] = stats + (datname, )
                with open(datname, 'w') as f:
                    for l in points:
                        xoffset = config['xoffset'] if 'xoffset' in config else 0
                        f.write('%f %f\n' % (xoffset + l[0], l[1]))
        
                dot_count = 4
                step = math.floor(len(points) / dot_count)
                # plot_mark_offset in [1, len(algorithms)]
                first = math.ceil(plot_mark_offset / (len(algorithms) + 1) * step)
                with open(datname + '.dot', 'w') as f:
                    for l in points[first::step]:
                        xoffset = config['xoffset'] if 'xoffset' in config else 0
                        f.write('%f %f\n\n' % (xoffset + l[0], l[1]))
        
        statname = ('stats/' + prefix + '-' + title.replace(' ', '-') + '.txt')
        delim = ''
        with open(statname, 'w') as f:
            for metric in metrics:
                f.write(delim)
                delim = '\n'
                f.write('%s:\n' % metric[0])
                for algorithm in algorithms:
                    i = info[metric[0]][algorithm[0]]
                    f.write('%s: %f %f %f\n' % (algorithm[0], i[0], i[1], i[2]))

        xranges = subfig[3][:] if len(subfig) > 3 else None
        mi = -1
        for metric in metrics:
            mi += 1
            config = metric[2] if len(metric) > 2 else {}
            pdfname = ('figures/' + prefix + '-' +
                       title.replace(' ', '-') + '-' +
                       metric[0].replace(' ', '-') +
                       '.pdf')
            key = config['key'] if 'key' in config else 'bottom right'
            xtics = str(config['xtics']) if 'xtics' in config else 'autofreq'
            #xlabel = title + ' ' + metric[0]
            xlabel = metric[0]
            if 'time' in xlabel:
                xlabel += ' (s)'
            elif 'bitrate' in xlabel:
                xlabel += ' (kbps)'
            if xranges:
                xrange = '[0:%f]' % xranges.pop(0)
            else:
                xrange = '[0:*]'
            plot_list = []
            point_types = [1, 2, 4, 6, 8, 10]
            pti = 0
            for algorithm in algorithms:
                pt = point_types[pti]
                pti += 1
                alg_pars = algorithm[2]
                if alg_pars.startswith('notitle'):
                    alg_pars = alg_pars[len('notitle'):]
                    # HACK
                    if isinstance(term, list) and term[mi] != None:
                        do_title = ' notitle '
                    else:
                        do_title = ' title "' + algorithm[0] + '" '
                else:
                    do_title = ' title "' + algorithm[0] + '" '
                datname = info[metric[0]][algorithm[0]][-1]
                plot_list += ['"' + datname + '" notitle ' + alg_pars + ' lw 2']
                plot_list += ['"' + datname + '.dot" ' + do_title +
                              ' with linespoints pt ' + str(pt) + ' ' + alg_pars + ' lw 2']

            trm = term[mi] if isinstance(term, list) else term
            if trm == None:
                trm = 'pdf size 2.3, 1.75 font ",16"'

            plotting = '''set term ''' + trm + '''
set bmargin 3.5

set style data lines
set key ''' + key + '''

set xlabel "''' + xlabel + '''"
set xtics ''' + xtics + '''

set xrange ''' + xrange + '''

set output "''' + pdfname + '''"

plot ''' + ', '.join(plot_list) + '''

set output
'''
            #subprocess.run('gnuplot', input = plotting.encode('ascii'))
            t = threading.Thread(target = thread_run_gnuplot, args = (plotting, ))
            plotting_threads.append(t)
            t.start()

        print('\b' * (len(title) + 1), end = '')
        print(' '  * (len(title) + 1), end = '')
        print('\b' * (len(title) + 1), end = '')

    for t in plotting_threads:
        t.join()

    print('\b' * (len(prefix) + 1), end = '')
    print(' '  * (len(prefix) + 1), end = '')
    print('\b' * (len(prefix) + 1), end = '')

def figure12_write_network():
    with open('tmp/network.json', 'w') as f:
        f.write('[ {"duration_ms": 60000, "bandwidth_kbps": 8000, "latency_ms":  0} ]')


def figure6a():
    figure12_write_network()

    completed = subprocess.run(['python3', './sabre-mmsys18.py', '-v',
                                '-m', 'bbb.json', '-n', 'tmp/network.json',
                                '-a', 'bola', '-ab'],
                               stdout = subprocess.PIPE)
    basic = completed.stdout.decode('ascii')

    completed = subprocess.run(['python3', './sabre-mmsys18.py', '-v',
                                '-m', 'bbb.json', '-n', 'tmp/network.json',
                                '-a', 'bolae'],
                               stdout = subprocess.PIPE)
    bolapl = completed.stdout.decode('ascii')

    fig1 = []
    for out in [basic, bolapl]:
        fig = []
        for line in out.split('\n'):
            if not '[' in line or 'Network' in line:
                continue
            l = line.split()
            
            index = int(l[1].split(':')[0])
            quality = int(l[2].split('=')[1])
            #print('%d %d' % (index, quality))
            fig += [(index * 3, bbb['bitrates_kbps'][quality])]
            fig += [((index + 1) * 3, bbb['bitrates_kbps'][quality])]
            if index == 9:
                break

        fig1 += [fig]

    for i in [0, 1]:
        name = 'fig1%s.dat' % ['a', 'b'][i]
        with open('tmp/%s' % name, 'w') as f:
            for l in fig1[i]:
                f.write('%f %f\n' % (l[0], l[1]))

    plotting = '''set term pdf size 1.9, 1.75 font ",16"
set bmargin 3.5

set style data lines
set yrange[0:6500]

set xlabel 'play time (s)'
set ylabel 'bitrate (kbps)'

set xtics 10

#set key bottom right
set key out top center

set output "figures/fig6a.pdf"

#plot "tmp/fig1a.dat" title "BOLA" lc 7 dt 4 lw 2, "tmp/fig1b.dat" title "BOLA-PL" lc 6 lw 2
plot "tmp/fig1a.dat" title "BOLA" lc 7 dt 4 lw 2, "tmp/fig1b.dat" notitle lc 6 lw 2

set output
'''
    subprocess.run('gnuplot', input = plotting.encode('ascii'))

def figure6b():
    figure12_write_network()

    completed = subprocess.run(['python3', './sabre-mmsys18.py', '-v',
                                '-m', 'bbb.json', '-n', 'tmp/network.json',
                                '-s', '120', '180',
                                '-a', 'bola', '-ab'],
                               stdout = subprocess.PIPE)
    basic = completed.stdout.decode('ascii')

    completed = subprocess.run(['python3', './sabre-mmsys18.py', '-v',
                                '-m', 'bbb.json', '-n', 'tmp/network.json',
                                '-s', '120', '180',
                                '-a', 'bolae'],
                               stdout = subprocess.PIPE)
    bolapl = completed.stdout.decode('ascii')

    fig2 = []
    for out in [basic, bolapl]:
        fig = []
        for line in out.split('\n'):
            if not '[' in line or 'Network' in line:
                continue
            l = line.split()
            index = int(l[1].split(':')[0])
            quality = int(l[2].split('=')[1])
            if index < 35:
                continue
            if index == 60:
                fig += [None]
            #print('%d %d' % (index, quality))
            fig += [(index * 3, bbb['bitrates_kbps'][quality])]
            fig += [((index + 1) * 3, bbb['bitrates_kbps'][quality])]
            if index == 69:
                break

        fig2 += [fig]

    for i in [0, 1]:
        name = 'fig2%s.dat' % ['a', 'b'][i]
        with open('tmp/%s' % name, 'w') as f:
            for l in fig2[i]:
                if l == None:
                    f.write('\n')
                else:
                    f.write('%f %f\n' % (l[0], l[1]))

    plotting = '''set term pdf size 1.47, 1.75 font ",16"
set bmargin 3.5

set style data lines
set xrange[180:]
set yrange[0:6500]

set xlabel 'play time (s)'

#set ylabel 'bitrate (kbps)'
set ytics format ""
set xtics 10

#set key bottom right
set key out top center

set output "figures/fig6b.pdf"

#plot "tmp/fig2a.dat" title "BOLA" lc 7 dt 4 lw 2, "tmp/fig2b.dat" title "BOLA-PL" lc 6 lw 2
plot "tmp/fig2a.dat" notitle lc 7 dt 4 lw 2, "tmp/fig2b.dat" title "BOLA-PL" lc 6 lw 2

set output
'''
    subprocess.run('gnuplot', input = plotting.encode('ascii'))

def figure_1_4():

    with open('tmp/egbuf.dat', 'w') as f:
        f.write('''0 1000
5 1000
5 2500
10 2500
10 5000
15 5000
15 0
18 0
''')

    with open('tmp/lowbufa.dat', 'w') as f:
        f.write('''0 0
0 230
3.534 230
3.534 331
3.843 331
3.843 477
4.153 477
4.153 688
4.462 688
4.462 991
4.771 991
4.771 1427
5.081 1427
5.081 2056
5.390 2056
5.390 2962
5.759 2962
5.759 5027
6.075 5027
6.075 6000
7.000 6000
7.000 0
10 0
''')
    with open('tmp/lowbufb.dat', 'w') as f:
        f.write('''0 0
0 230
11.048 230
11.048 331
13.284 331
13.284 477
15.527 477
15.527 688
17.770 688
17.770 991
20.007 991
20.007 1427
22.244 1427
22.244 2056
24.483 2056
24.483 2962
27.150 2962
27.150 5027
29.441 5027
29.441 6000
36.132 6000
36.132 0
40 0
''')

    plotting1 = '''set term pdf size 3.35, 1.5 font ",16"
set bmargin 3.5

set style data lines

set xlabel 'buffer level (s)'
set ylabel 'bitrate (kbps)'

set output "figures/fig-1.pdf"

set xrange[0:18]
set yrange[0:6000]

#set arrow from 12.5,3500 to 5,0
#set arrow from 12.5,3500 to 10,0
#set arrow from 12.5,3500 to 15,0
#
#set arrow from 2,4000 to 0,5000
#set arrow from 2,4000 to 0,2500
#set arrow from 2,4000 to 0,1000

set arrow from 5,0 to 5,1000 nohead dt 2
set arrow from 10,0 to 10,2500 nohead dt 2

set arrow from 0,2500 to 5,2500 nohead dt 2
set arrow from 0,5000 to 10,5000 nohead dt 2

plot "tmp/egbuf.dat" lc 7 lw 2 notitle

set output
'''

    plotting2 = '''set term pdf size 3.35, 1.5 font ",16"
set bmargin 3.5

set style data lines

set xlabel 'buffer level (s)'
set ylabel 'bitrate (kbps)'

set output "figures/fig-4a.pdf"

set xrange[0:10]
set yrange[0:6500]

plot "tmp/lowbufa.dat" lc 7 lw 2 notitle

set output

set output "figures/fig-4b.pdf"

set xrange[0:40]
set yrange[0:6500]

set grid noxtics noytics noztics front
set style rect fc lt -1 fs solid 0.25 noborder
set obj rect from 16, 0 to 26, 6500
set arrow from 0.1,3500 to 15.9,3500 heads
set arrow from 16.1,3500 to 25.9,3500 heads
set label "virtual\\nplaceholder\\nsegments" at 8,5600 center
set label "actual\\nvideo\\nsegments" at 21,5600 center

plot "tmp/lowbufb.dat" lc 6 lw 2 notitle

set output
'''
    subprocess.run('gnuplot', input = plotting1.encode('ascii'))
    subprocess.run('gnuplot', input = plotting2.encode('ascii'))

def figure_7_10():
    subfigs = [
        #('12dash vod'   , '12dash', ['-m', 'bbb.json'  , '-b', '25']),
        #('3Glogs vod'   , '3Glogs', ['-m', 'bbb.json'  , '-b', '25']),
        ('4G VOD'   , '4Glogs', ['-m', 'bbb4k.json', '-b', '25']),
        #('12dash live10', '12dash', ['-m', 'bbb.json'  , '-b', '10']),
        #('3G LIVE 10s', '3Glogs', ['-m', 'bbb.json'  , '-b', '10']),
        #('4Glogs live10', '4Glogs', ['-m', 'bbb4k.json', '-b', '10']),
        #('12dash live5' , '12dash', ['-m', 'bbb.json'  , '-b', '5' ]),
        #('3Glogs live5' , '3Glogs', ['-m', 'bbb.json'  , '-b', '5' ]),
        #('4Glogs live5' , '4Glogs', ['-m', 'bbb4k.json', '-b', '5' ]),
    ]

    metrics = [
        #('rebuffer'   , 'rebuffer ratio'),
        #('oscillation', 'time average bitrate change'),
        #('bitrate'    , 'time average played bitrate'),
        ('reaction time'     , 'rampup time', {'key': 'out top center horizontal', 'xtics': 10}),
    ]

    prefix = 'fig7a'
    algorithms = [
        ('BOLA'   , ['-ao', '-a', 'bola', '-ab']    , 'lc 7'),
        ('BOLA-PL', ['-ao', '-a', 'bolae', '-noibr'], 'notitle lc 6'),
    ]

    term = 'pdf size 1.8, 1.75 font ",16"'
    do_figure(prefix, subfigs, algorithms, metrics, term = term)

    prefix = 'fig7b'
    algorithms = [
        ('BOLA'   , ['-ao', '-a', 'bola', '-ab'    , '-s', '120', '180'], 'notitle lc 7'),
        ('BOLA-PL', ['-ao', '-a', 'bolae', '-noibr', '-s', '120', '180'], 'lc 6'),
    ]
    term = 'pdf size 1.5, 1.75 font ",16"\nset ytics format ""'
    do_figure(prefix, subfigs, algorithms, metrics, term = term)


    prefix = 'fig10a'
    algorithms = [
        ('BOLA'         , ['-ao', '-a', 'bola', '-ab'], 'lc 4'),
        ('TPUT'   , ['-ao', '-a', 'throughput'], 'lc 2'),
        ('DYNAMIC'      , ['-ao', '-a', 'dynamic', '-ab'], 'notitle lc 1'),
    ]
    term = 'pdf size 1.8, 1.75 font ",16"'
    do_figure(prefix, subfigs, algorithms, metrics, term = term)

    prefix = 'fig10b'
    algorithms = [
        ('BOLA'         , ['-ao', '-a', 'bola', '-ab', '-s', '120', '180'], 'notitle lc 4'),
        ('TPUT'   , ['-ao', '-a', 'throughput', '-s', '120', '180'], 'notitle lc 2'),
        ('DYNAMIC'      , ['-ao', '-a', 'dynamic', '-ab', '-s', '120', '180'], 'lc 1'),
    ]
    term = 'pdf size 1.5, 1.75 font ",16"\nset ytics format ""'
    do_figure(prefix, subfigs, algorithms, metrics, term = term)

def figure8():
    prefix = 'fig8'

    algorithms = [
        ('BOLA',    ['-a', 'bola',  '-ao', '-ab'],    ' lc 4'),
        ('BOLA-PL', ['-a', 'bolae', '-ao', '-noibr'], ' lc 7'),
        ('BOLA-E' , ['-a', 'bolae', '-ao'          ], ' lc 6'),
    ]

    metrics = [
        ('rebuffer ratio'   , 'rebuffer ratio', {'xtics' : 0.1}),
        ('average bitrate oscillation', 'time average bitrate change', {'xtics': 150}),
        ('average bitrate'    , 'time average played bitrate', {'xtics': 500}),
    ]

    subfigs = [
        ('3G Live 10s'   , '3Glogs', ['-m', 'bbb.json'  , '-b', '10'], [0.6, 600, 2000]),
    ]

    do_figure(prefix, subfigs, algorithms, metrics)

#    metrics = [
#        ('rebuffer ratio'   , 'rebuffer ratio'),
#        ('average bitrate oscillation', 'time average bitrate change', {'xtics': 500}),
#        ('average bitrate'    , 'time average played bitrate', {'xtics': 10000}),
#    ]
#
#    subfigs = [
#        ('4G Live 10s', '4Glogs', ['-m', 'bbb4k.json', '-b', '10']),
#        ('4G VOD', '4Glogs', ['-m', 'bbb4k.json', '-b', '25']),
#    ]
#
#    do_figure(prefix, subfigs, algorithms, metrics)


def figure11():
    prefix = 'fig11'

    subfigs = [
        #('12dash vod'   , '12dash', ['-m', 'bbb.json'  , '-b', '25']),
        #('3Glogs vod'   , '3Glogs', ['-m', 'bbb.json'  , '-b', '25']),
        ('4G VOD'   , '4Glogs', ['-m', 'bbb4k.json', '-b', '25'], [0.1, 2200, 34000]),
        #('12dash live10', '12dash', ['-m', 'bbb.json'  , '-b', '10']),
        #('3Glogs live10', '3Glogs', ['-m', 'bbb.json'  , '-b', '10']),
        ('4G Live 10s', '4Glogs', ['-m', 'bbb4k.json', '-b', '10'], [0.1, 4600, 31500]),
        #('12dash live5' , '12dash', ['-m', 'bbb.json'  , '-b', '5' ]),
        #('3Glogs live5' , '3Glogs', ['-m', 'bbb.json'  , '-b', '5' ]),
        #('4Glogs live5' , '4Glogs', ['-m', 'bbb4k.json', '-b', '5' ], [0.1, 4000, 35000]),
    ]

    algorithms = [
        #('BOLA-E'       , ['-ao', '-a', 'bolae'      ], 'lc 7'),
        #('DYNAMIC-DASH' , ['-ao', '-a', 'dynamicdash'], 'lc 1'),
        ('BOLA'         , ['-ao', '-a', 'bola'      ], 'lc 4'),
        ('THROUGHPUT'   , ['-ao', '-a', 'throughput'], 'lc 2'),
        ('DYNAMIC'      , ['-ao', '-a', 'dynamic'   ], 'lc 1'),
    ]

    metrics = [
        ('rebuffer ratio'   , 'rebuffer ratio'),
        ('average bitrate oscillation', 'time average bitrate change', {'xtics': 1000, 'key': 'bottom right font ",12"'}),
        ('average bitrate'    , 'time average played bitrate', {'xtics': 10000, 'key': 'bottom right font ",12"'}),
    ]

    do_figure(prefix, subfigs, algorithms, metrics)

def figure_12_13():

    prefix = '12_13'

    subfigs = [
        ('FCC SD', 'sd_fs', ['-m', 'bbb.json'  , '-b', '25'], [0.01, 450, 4500, 120]),
    ]

    algorithms = [
        ('BOLA-E'    , ['-ao', '-r', 'none', '-a', 'bolae'  , '-rmp', '9', '-ml', '180'], 'lc 7'),
        ('BOLA-E-FS' , ['-ao', '-r', 'left', '-a', 'bolae'  , '-rmp', '9', '-ml', '180'], 'lc 1'),
        ('DYNAMIC'   , ['-ao', '-r', 'none', '-a', 'dynamic', '-rmp', '9', '-ml', '180'], 'notitle lc 3'),
        ('DYNAMIC-FS', ['-ao', '-r', 'left', '-a', 'dynamic', '-rmp', '9', '-ml', '180'], 'notitle lc 4'),
    ]

    metrics = [
        ('rebuffer ratio'     , 'rebuffer ratio'),
        ('average bitrate oscillation'  , 'time average bitrate change', {'xtics': 150}),
        ('average bitrate'      , 'time average played bitrate',
         {'key': 'top left reverse Left font ",12"', 'xtics': 1500}),
        ('reaction time', 'rampup time', {'xoffset': -60, 'key': 'out top center vertical', 'xtics': 40}),
    ]

    term = 'pdf size 1.8, 1.75 font ",16"\n'
    do_figure(prefix, subfigs, algorithms, metrics, term = [None, None, None, term])

    prefix = '12_13'

    subfigs = [
        ('FCC HD', 'hd_fs', ['-m', 'bbb4k.json', '-b', '25'], [0.01, 1200, 12000, 120]),
    ]

    algorithms = [
        ('BOLA-E'    , ['-ao', '-r', 'none', '-a', 'bolae'  , '-rmp', '4', '-ml', '180'], 'notitle lc 7'),
        ('BOLA-E-FS' , ['-ao', '-r', 'left', '-a', 'bolae'  , '-rmp', '4', '-ml', '180'], 'notitle lc 1'),
        ('DYNAMIC'   , ['-ao', '-r', 'none', '-a', 'dynamic', '-rmp', '4', '-ml', '180'], 'lc 3'),
        ('DYNAMIC-FS', ['-ao', '-r', 'left', '-a', 'dynamic', '-rmp', '4', '-ml', '180'], 'lc 4'),
    ]

    metrics = [
        ('rebuffer ratio'     , 'rebuffer ratio'),
        ('average bitrate oscillation'  , 'time average bitrate change', {'xtics': 400}),
        ('average bitrate'      , 'time average played bitrate',
         {'key': 'top left reverse Left font ",12"', 'xtics': 4000}),
        ('reaction time', 'rampup time', {'xoffset': -60, 'key': 'out top center vertical', 'xtics': 40}),
    ]

    term = 'pdf size 1.5, 1.75 font ",16"\nset ytics format ""'
    do_figure(prefix, subfigs, algorithms, metrics, term = [None, None, None, term])
    
if __name__ == '__main__':

    bbb = load_json('bbb.json')
    bbb4k = load_json('bbb4k.json')

    os.makedirs('tmp', exist_ok = True)
    os.makedirs('figures', exist_ok = True)
    os.makedirs('stats', exist_ok = True)

    figure6a()
    figure6b()
    figure_1_4()
    figure_7_10()
    figure8()
    figure11()
    figure_12_13()
