#!/bin/sh

# run params.
ZU3_IP=
tun_bin=./`basename $0 .sh`
isp=
isp_calibin=
isp_lname=
isp_calibration=tools/calibration/tuning_test
i3a_bin=
#i3a_bin=tools/isp_3a.elf
camera=y
cam_config=
cam_config_p=
cam_index=
cam_index_p=
cam_port=
cam_port_p=
pipe_num=
vio_cfgs_p=
runtime=
runtime_p=
monitor=y
monitor_p=
vin_deinit=
vin_deinit_p=
loglevel=
printk=
editcam=
editmipi=
editcim=
cimtype=
cimdump=
cimshow=
isptype=
showcfg=
notrun=
hts=
vts=
fps=
extra_support="os8a10\|imx290\|ar0233\|ar0820"
no3a_filter="dts"
fps_support="vpg"
#cam_config_dft=cfg/cam/hb_j5dev.json
cam_config_dft=./hb_j5dev.json
vpm_cases_cfgdir=./
showvpm=
vpm_case=
env_setup=
env_force=

# args.
for arg in $@; do
	if [ "${arg}" == "-h" ]; then
cat <<EOF
`basename $0` [.json] [.so] [#] [0x#] [p#] [r#] [l#] [L#] [h#] [v#] [f#] [D#] [A#] [t#] [d] [s] [e] [a] [m] [i] [x] [y] [C/X/B/M] [ip] [n] [-h]
  hb_.json -- cam config json
  xxx.json -- cim config json
  *.so     -- isp calib so
  #        -- cam index
  0x#      -- cam port mask
  p#       -- pipe num
  r#       -- runtime seconds
  l#       -- userspace loglevel
  L#       -- kernel loglevel
  h#       -- hts value for config_x, support ${extra_support}
  v#       -- vts value for config_x, support ${extra_support}
  f#       -- fps select for config_x, support ${fps_support}
  D#       -- vin_deinit mipi# first
  A#       -- isp output type
  t#       -- cim_type for cimdma
  d#       -- cim_dump for cimdma
  s#       -- cim_show for cimdma
  e        -- env setup force(as ${env_setup} f)
  a        -- edit cam json file
  m        -- edit mipi json file
  i        -- edit cim json file
  x        -- show comment of json config
  y        -- show app vpm cases list
  C/X/B/M  -- no cam/isp/isp_cal/hobotplayer
  a.b.c.d  -- zu3 ip if not default ${ZU3_IP}
  n        -- not run, only show info
  -h       -- show this help tips
EOF
		exit 0
	elif [ "${arg:0-5}" == ".json" ]; then
		jfile=`basename $arg`
		if [ "${jfile:0:3}" == "hb_" ]; then
			cam_config="${arg}"
		else
			echo "${arg} not cam json"
		fi
	elif [ "${arg:0-3}" == ".so" ]; then
		isp_lname=${arg}
	elif [ "${arg/0x[0-9a-fA-F]*/}" == "" ]; then
		cam_port="${arg}"
	elif [ "${arg//[0-9]/}" == "..." ]; then
		ZU3_IP=${arg}
	elif [ "${arg/[0-9]*/}" == "" ]; then
		cam_index="${arg}"
	elif [ "${arg/p[0-9]*/}" == "" ]; then
		pipe_num="${arg/r/}"
	elif [ "${arg/r[0-9]*/}" == "" ]; then
		runtime="${arg/r/}"
	elif [ "${arg/l[0-9]*/}" == "" ]; then
		loglevel="${arg/l/}"
	elif [ "${arg/L[0-9]*/}" == "" ]; then
		printk="${arg/L/}"
	elif [ "${arg/h[0-9]*/}" == "" ]; then
		hts="${arg/h/}"
	elif [ "${arg/v[0-9]*/}" == "" ]; then
		vts="${arg/v/}"
	elif [ "${arg/f[0-9]*/}" == "" ]; then
		fps="${arg/f/}"
	elif [ "${arg/D[0-9]*/}" == "" ]; then
		vin_deinit="${arg/D/}"
	elif [ "${arg/A[0-9]*/}" == "" ]; then
		isptype="${arg/A/}"
	elif [ "${arg}" == "A" ]; then
		isptype=4
	elif [ "${arg/t[0-9]*/}" == "" ]; then
		cimtype="${arg/t/}"
	elif [ "${arg/d[0-9\-]*/}" == "" ]; then
		cimdump="${arg/d/}"
	elif [ "${arg/s[0-9]*/}" == "" ]; then
		cimshow="${arg/s/}"
	elif [ "${arg}" == "t" ]; then
		cimtype=0
	elif [ "${arg}" == "d" ]; then
		cimdump=1
	elif [ "${arg}" == "s" ]; then
		cimshow=2
	elif [ "${arg}" == "a" ]; then
		editcam=y
		notrun=y
	elif [ "${arg}" == "m" ]; then
		editmipi=0
		notrun=y
	elif [ "${arg/m[0-9]*/}" == "" ]; then
		editmipi="${arg/m/}"
		notrun=y
	elif [ "${arg}" == "i" ]; then
		editcim=0
		notrun=y
	elif [ "${arg/i[0-9]*/}" == "" ]; then
		editcim="${arg/i/}"
		notrun=y
	elif [ "${arg}" == "x" ]; then
		showcfg=y
	elif [ "${arg}" == "y" ]; then
		showvpm=y
	elif [ "${arg}" == "e" ]; then
		env_force=f
	elif [ "${arg}" == "n" ]; then
		notrun=y
	elif [ "${arg}" == "C" ]; then
		camera=
	elif [ "${arg}" == "X" ]; then
		isp=
	elif [ "${arg}" == "B" ]; then
		isp_cal=
	elif [ "${arg}" == "M" ]; then
		monitor=
	elif [ -d ${vpm_cases_cfgdir}/${arg} ]; then
		vpm_case=${vpm_cases_cfgdir}/${arg}
	else
		vpm_case=`find ${vpm_cases_cfgdir} -type d -name "${arg}"`
		if [ -z "${vpm_case}" ]; then
			echo "${arg} not support"
			exit 1
		fi
	fi
done

# y for vpm show.
if [ ! -z "${showvpm}" ]; then
	if [ -z "${vpm_case}"  ]; then
		vpm_case=${vpm_cases_cfgdir}
	fi
	echo "vpm cases(${vpm_case}):"
	find ${vpm_case} -type d -mindepth 1 -maxdepth 1 |sed -e "s#${vpm_case}/#  #g"
	exit 0
fi

# is vpm case?
if [ -z "${cam_config}" ] && [ ! -z "${vpm_case}" ]; then
	cam_config=${vpm_case}/hb_j5dev.json
	if [ ! -f ${cam_config} ]; then
		echo "${cam_config} not exist"
		exit 1
	fi
fi

# x for config show.
if [ ! -z "${showcfg}" ]; then
	if [ -z "${cam_config}" ]; then
		cam_config=${cam_config_dft}
	fi
	if [ -z "${cam_index}" ]; then
		cam_index=0
	fi
	if [ ! -f ${cam_config} ]; then
		echo "${cam_config} not exist"
		exit 1
	fi
	config_num=`cat ${cam_config} |grep "\"config_number\"" |awk -F ':' '{print $2}' |awk -F ',' '{print $1}'`
	if [ -z "${config_num}" ]; then
		echo "${cam_config} config_number empty"
		exit 1
	fi
	echo "${cam_config} ${config_num} configs:"
	config_n=0
	while [ ${config_n} -lt ${config_num} ]; do
		config_comment=`cat ${cam_config} |grep -A 3 "\"config_${config_n}\"" |grep "\"comment\"" |awk -F ':' '{print $2}' |awk -F '"' '{print $2}'`
		if [ -z "${config_comment}" ]; then
			config_comment="unknown"
		fi
		if [ ${config_n} -eq ${cam_index} ]; then
			echo "* config_${config_n} - ${config_comment}"
		else
			echo "  config_${config_n} - ${config_comment}"
		fi
		config_n=`expr ${config_n} + 1`
	done
	exit 0
fi

# tun_bin check.
if [ "$(dirname ${tun_bin})" == "." ] && [ "${tun_bin:0:1}" != "." ]; then
	tun_bin=./${tun_bin}
fi
if [ ! -f ${tun_bin} ]; then
	echo "${tun_bin} not exist"
	exit 1
fi

if [ ! -z "${cam_config}" ]; then
	cam_config_p=" -c ${cam_config}"
else
	cam_config_p=" -c ${cam_config_dft}"
	cam_config=${cam_config_dft}
fi
if [ ! -z "${cam_index}" ]; then
	cam_index_p=" -i ${cam_index}"
else
	cam_index=0
fi
if [ -f ${cam_config} ]; then
	config_line=`cat ${cam_config} |grep -n "\"config_${cam_index}\"" |awk -F ':' '{print $1}'`
	config_next=`cat ${cam_config} |grep -n "\"config_$((${cam_index} + 1))\"" |awk -F ':' '{print $1}'`
	if [ -z "${config_next}" ]; then
		this_config=`sed -n "${config_line},$$p" ${cam_config}`
	else
		this_config=`sed -n "${config_line},$((${config_next} - 1))p" ${cam_config}`
	fi
	config_comment=`echo "${this_config}" |grep "\"comment\"" |awk -F ':' '{print $2}' |awk -F '"' '{print $2}'`
	if [ ! -z "${editcim}" ]; then
		index=$((${editcim} + 1))
	else
		index=1
	fi
	cim_config=`echo "${this_config}" |grep "\"data_path\"" |sed -n "${index}p" |awk -F ':' '{print $2}' |awk -F '"' '{print $2}'`
	if [ ! -z "${editmipi}" ]; then
		index=$((${editmipi} + 1))
	else
		index=1
	fi
	cfg_fps=`echo "${this_config}" |grep "\"fps\"" |sed -n "${index}p" |awk -F '[:,]' '{print $2}'`
	cfg_res=`echo "${this_config}" |grep "\"resolution\"" |sed -n "${index}p" |awk -F '[:,]' '{print $2}'`
	cfg_path=`echo "${this_config}" |grep "\"config_path\"" |sed -n "${index}p" |awk -F '[:,]' '{print $2}' |awk -F '"' '{print $2}'`
	if [ ! -z "${cfg_fps}" ] && [ ! -z "${cfg_res}" ] && [ ! -z "${cfg_path}" ] ; then
		mipi_config=`printf ${cfg_path} ${cfg_fps} ${cfg_res}`
	fi
	for x in ${config_comment}; do
		if [ -z "${config_snr}" ]; then
			config_snr=${x/(*/}
		elif [ "${x}" == "pwl" ] || [ "${x}" == "linear" ] || [ "${x:0:3}" == "dol" ]; then
			config_mode=${x}
		elif [ "${x}" == "normal" ]; then
			config_mode="linear"
		elif [ "$x" != "x" ] && [ "${x//[0-9]/}" == "x" ]; then
			config_res=${x}
		fi
	done
fi
if [ ! -z "${cam_port}" ]; then
	cam_port_p=" -o $((${cam_port}))"
elif [ -f ${cam_config} ] && [ ! -z "${this_config}" ]; then
	# auto cal port_mask by port_number.
	cam_port_n=`echo "${this_config}" |grep "\"port_number\"" |head -n 1 |awk -F ':' '{print $2}' |awk -F ',' '{print $1}'`
	if [ ! -z "${cam_port_n}" ]; then
		cam_port=$(((1 << ${cam_port_n}) - 1))
		cam_port_p=" -o $((${cam_port}))"
	fi
fi
if [ ! -z "${runtime}" ]; then
	runtime_p=" -r ${runtime}"
fi
if [ ! -z "${vin_deinit}" ]; then
	vin_deinit_p=" -D ${vin_deinit}"
fi
if [ -z "${camera}" ]; then
	cam_config_p=" -c #"
	cam_index_p=
	cam_port_p=
	vin_deinit_p=
fi
if [ ! -z "${cimtype}" ]; then
	cim_p="${cim_p} -t ${cimtype}"
	if [ ! -z "${cimdump}" ]; then
		cim_p="${cim_p} -d ${cimdump}"
	fi
	if [ ! -z "${cimshow}" ]; then
		cim_p="${cim_p} -s ${cimshow}"
	fi
	vio_cfgs_p=" -x 0"
	monitor_p=" -H #"
else
	if [ ! -z "${cam_config}" ]; then
		if [ -z "${pipe_num}" ]; then
			pipe_num=1
		fi
		if [ -z "${config_res}" ]; then
			config_res="1920x1080"
		fi
		if [ "${config_mode}" == "pwl" ]; then
			config_m=1
		else
			config_m=0
		fi
		if [ -z "${isp}" ]; then
			vio_cfgs_p=" -x 0"
		else
			vio_cfgs_p=" -x :${config_res}@${config_m}"
			if [ ! -z "${isptype}" ]; then
				vio_cfgs_p="${vio_cfgs_p}&${isptype}"
				if [ ${isptype} -le 5 ] || [ ${isptype} -ge 11 ]; then
					vio_cfgs_p="${vio_cfgs_p} -H /system/etc/vio_tool/dump_raw.json"
				fi
			fi
			if [ ! -z "${isp_calibin}" ] && [ ! -z "${isp_lname}" ]; then
				vio_cfgs_p="${vio_cfgs_p} -C ${isp_lname}"
			fi
		fi
	fi
fi
if [ -z "${monitor}" ]; then
	monitor_p=" -H #"
fi

# h# v# f# parse.
if [ -f ${cam_config} ]; then
	if [ ! -z "${hts}" ] || [ ! -z ${vts} ]; then
		if [ ! -z "`echo ${config_comment} |grep "${extra_support}"`" ]; then
			if [ -z "${hts}" ]; then
				hts=0
			fi
			if [ -z "${vts}" ]; then
				vts=0
			fi
			if [ ${vts} -gt 32767 ]; then
				vts=32767
			fi
			extra=$(((${vts} << 16) + ${hts}))
		else
			echo "config_${cam_index} not support h# v#"
		fi
	fi
	if [ ! -z "${fps}" ]; then
		if [ -z "`echo ${config_comment} |grep "${fps_support}"`" ]; then
			echo "config_${cam_index} not support f#"
			fps=
		fi
	fi
	if [ ! -z "${i3a_bin}" ]; then
		if [ ! -z "`echo ${config_comment} |grep "${no3a_filter}"`" ]; then
			echo "config_${cam_index} no 3a"
			i3a_bin=
		fi
	fi
else
	extra=
	fps=
fi

# VINRPC zu3 ip.
if [ -z "${VINRPC_HOST}" ] && [ ! -z "${ZU3_IP}" ]; then
	VINRPC_HOST=${ZU3_IP}
fi

# show without run.
if [ ! -z "${editcam}" ]; then
	if [ ! -z "${cam_config}" ] && [ -f ${cam_config} ]; then
		vi ${cam_config} -c ${config_line}
	fi
fi
if [ ! -z "${editmipi}" ]; then
	if [ ! -z "${mipi_config}" ] && [ -f ${mipi_config} ]; then
		vi ${mipi_config} -c 20
	fi
fi
if [ ! -z "${editcim}" ]; then
	if [ ! -z "${cim_config}" ] && [ -f ${cim_config} ]; then
		vi ${cim_config} -c 11
	fi
fi

echo "========================================================================="
echo "[$(date '+%Y-%M-%d %H:%m:%S')] run ${cam_index} for ${cam_config}(${config_line}):"
if [ ! -z "${notrun}" ]; then
	echo "${tun_bin}${cam_config_p}${cam_index_p}${cam_port_p}${cim_p}${vin_deinit_p}${vio_cfgs_p}${runtime_p}${monitor_p}"
	if [ ! -z "${i3a_bin}" ] && [ -e ${i3a_bin} ]; then
		echo "./${i3a_bin} &"
	fi
	if [ ! -z "${config_line}" ]; then
		echo ">> CONFIG=${cam_config} > ${config_line}: config_${cam_index}"
	fi
	if [ ! -z "${config_comment}" ]; then
		echo ">> COMMENT=${config_comment}"
	fi
	if [ ! -z "${VINRPC_HOST}" ]; then
		echo ">> VINRPC_HOST=${VINRPC_HOST}"
	fi
	if [ ! -z "${loglevel}" ]; then
		echo ">> LOGLEVEL=${loglevel}"
	elif [ ! -z "${LOGLEVEL}" ]; then
		echo ">> LOGLEVEL=${LOGLEVEL}"
	fi
	if [ ! -z "${printk}" ]; then
		echo ">> printk=${printk}"
	fi
	if [ ! -z "${extra}" ]; then
		echo ">> HTS=${hts} VTS=${vts} --> extra_mode=${extra}"
	fi
	if [ ! -z "${fps}" ]; then
		echo ">> fps=${fps}"
	fi
	echo "-------------------------------------------------------------------------"
	echo ">> CAM (a): ${cam_config}"
	echo ">> CIM (i): ${cim_config}"
	echo ">> MIPI(m): ${mipi_config}"
	echo "-------------------------------------------------------------------------"
	exit 0
fi

# 3a kill.
if [ ! -z "${i3a_bin}" ] && [ -e ${i3a_bin} ] && [ ! -z "${monitor}" ]; then
	killall -9 `basename ${i3a_bin}`
fi

# unload calibration.
if [ ! -z "${isp_cal}" ] && [ ! -z "${isp_calibration}" ] && [ -e ${isp_calibration} ] && [ ! -z "${monitor}" ]; then
	if [ ! -z "${isp_lname}" ]; then
		isp_calibration_file=${isp_lname}
	else
		isp_calibration_file=tools/calibration/lib${config_snr}_${config_mode}.so
	fi
	if [ ! -z "${isp_calibration_file}" ] && [ -e ${isp_calibration_file} ]; then
		if [ ! -x ${isp_calibration} ]; then
			chmod +x ${isp_calibration}
		fi
		(./${isp_calibration} -p 0 -c 1)
		sleep 1
	fi
fi

# env setup.
if [ ! -z "${env_setup}" ] && [ -e ${env_setup} ]; then
	if [ ! -x ${env_setup} ]; then
		chmod +x ${env_setup}
	fi
	./${env_setup} ${env_force}
fi

# loglevel.
export LD_LIBRARY_PATH="`pwd`:${LD_LIBRARY_PATH}"
if [ ! -z "${loglevel}" ]; then
	export LOGLEVEL=${loglevel}
fi
if [ ! -z "${printk}" ]; then
	echo ${printk} >/proc/sys/kernel/printk
fi

# VINRPC config.
if [ ! -z "${VINRPC_HOST}" ]; then
	export VINRPC_HOST=${VINRPC_HOST}
fi

# h# v# f# do.
if [ ! -z "${extra}" ] || [ ! -z "${fps}" ]; then
	cam_index_n=`cat ${cam_config} |grep -n "\"config_${cam_index}\"" |awk -F ':' '{print $1}'`
	cam_index_nx=`cat ${cam_config} |grep -n "\"config_$((${cam_index} + 1))\"" |awk -F ':' '{print $1}'`
	if [ ! -z "${cam_index_nx}" ]; then
		cam_index_num="+$((${cam_index_nx} - ${cam_index_n} - 1))"
	else
		cam_index_num="$"
	fi
	if [ ! -z "${extra}" ]; then
		echo "${cam_config}/config_${cam_index}: extra_mode=${extra}"
		sed -e "${cam_index_n},${cam_index_num}s/\"extra_mode\":.*/\"extra_mode\":${extra},/" -i ${cam_config}
	fi
	if [ ! -z "${fps}" ]; then
		echo "${cam_config}/config_${cam_index}: fps=${fps}"
		sed -e "${cam_index_n},${cam_index_num}s/\"fps\":.*/\"fps\":${fps},/" -i ${cam_config}
	fi
fi

# load calibration.
if [ ! -z "${isp_cal}" ] && [ ! -z "${isp_calibration}" ] && [ -e ${isp_calibration} ] && [ ! -z "${monitor}" ]; then
	isp_calibration_file=tools/calibration/lib${config_snr}_${config_mode}.so
	if [ ! -z "${isp_calibration_file}" ] && [ -e ${isp_calibration_file} ]; then
		if [ ! -x ${isp_calibration} ]; then
			chmod +x ${isp_calibration}
		fi
		echo "isp calibrationg: ${isp_calibration_file}"
		(./${isp_calibration} -p 0 -c 0 -l ${isp_calibration_file})
		sleep 1
	fi
fi

# 3a run.
if [ ! -z "${i3a_bin}" ] && [ -e ${i3a_bin} ] && [ ! -z "${monitor}" ]; then
	if [ ! -x ${i3a_bin} ]; then
		chmod +x ${i3a_bin}
	fi
	(sleep 10; ./${i3a_bin}) &
fi

# tunning run.
if [ ! -x ${tun_bin} ]; then
	chmod +x ${tun_bin}
fi

echo "tun_bin: ${tun_bin}"
echo "cam_config_p: ${cam_config_p}"
echo "cam_index_p：${cam_index_p}"
echo "cam_port_p: ${cam_port_p}"
echo "cim_p: ${cim_p}"
echo "vin_deinit_p: ${vin_deinit_p}"
echo "vio_cfgs_p: ${vio_cfgs_p}"
echo "runtime_p: ${runtime_p}"
echo "monitor_p: ${monitor_p}"

${tun_bin}${cam_config_p}${cam_index_p}${cam_port_p}${cim_p}${vin_deinit_p}${vio_cfgs_p}${runtime_p}${monitor_p}
exit $?

