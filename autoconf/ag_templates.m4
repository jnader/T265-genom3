#
# Autoconf macros processing a --with-templates option for genom components
#

# --- AG_OPT_TEMPLATES(genom, input.gen) -----------------------------------
#
# Handle recursive template invocation.
#
AC_DEFUN([AG_OPT_TEMPLATES],
[
    ag_genom="$1"
    ag_input="$2"

    # enable --with-templates option
    AC_ARG_WITH(templates,
        AC_HELP_STRING([--with-templates], [comma separated list of templates.
            See genom3 -l for a list of valid choices. Passing a complete path to
            a template is also supported]),
        [ag_templates="$withval"])

    if test -n "$ag_templates"; then
        # user may want to pass options to templates
        AC_DISABLE_OPTION_CHECKING

        # we may need autoreconf
        AC_PATH_PROG(ag_autoreconf, [autoreconf], [no])

        # compute AG_TEMPLATES_SUBDIRS
        agdir=
        oIFS="$IFS"; IFS=","; set -- $ag_templates; IFS="$oIFS"
        for t in "$[@]"; do
            agdir=$agdir${agdir:+ }./${t#/}
        done
        AC_SUBST([AG_TEMPLATES_SUBDIRS],[$agdir])

        # compute recursive options
        _AG_ARGS_SUBDIRS
    fi
])


# --- AG_OUTPUT_TEMPLATES --------------------------------------------------
#
# A command to be run _after_ AC_OUTPUT, that generates/autoreconf templates
#
AC_DEFUN([AG_OUTPUT_TEMPLATES],
[
    echo "$ag_templates" | tr , '\n' | while read t; do
        if test "x$t" = x; then continue; fi
        tdir="./${t#/}"
        AC_MSG_NOTICE([configuring for $t])

        # run genom
        AC_MSG_NOTICE([running $ag_genom $t -C $tdir $ag_input])
        eval $ag_genom $t -C $tdir $ag_input
        if test $? != 0; then
            rm -rf "$tdir"
            AC_MSG_ERROR([cannot generate template $t], 2)
        fi

        # check for autoconf template (configure.ac)
        if test -f "$tdir/configure.ac"; then
            if ! test -f "$tdir/configure"; then
                AC_MSG_NOTICE([running autoreconf -vi for $t])
                if test "$ag_autoreconf" = no; then
                    AC_MSG_ERROR([autoreconf is missing, please install it], 2)
                fi
                (cd "$tdir" && $ag_autoreconf -vi)
                if test $? -ne 0; then
                    AC_MSG_ERROR([cannot reconfigure for $t], 2)
                fi
            fi

            # run configure
            if test -f "$tdir/configure"; then
                AC_MSG_NOTICE(
                    [running $SHELL configure $ag_sub_configure_args for $t])

                if test -f "$tdir/config.status"; then
                    # otherwise configure will complain...
                    rm "$tdir/config.status"
                fi

                # eval makes quoting arguments work
                eval "(cd \"$tdir\" && CONFIG_SHELL=$SHELL \
                     $SHELL configure $ag_sub_configure_args)" ||
		  AC_MSG_ERROR([configure failed for $t])
            fi
        fi

        AC_MSG_NOTICE([done configuring for $t])
    done
])


# --- _AG_ARGS_SUBDIRS -----------------------------------------------------
#
# This mimics _AC_OUTPUT_SUBDIRS by filtering unwanted args for recursive
# configure invocation
#
AC_DEFUN([_AG_ARGS_SUBDIRS],
[

    # Remove --srcdir, --disable-option-checking, --with-templates and
    # PKG_CONFIG_PATH arguments so they do not pile up. PKG_CONFIG_PATH is
    # added explicitly by config.status
    #
    ag_sub_configure_args=
    ag_prev=
    eval "set x $ac_configure_args"; shift
    for ag_arg in "$[@]"; do
        if test -n "$ag_prev"; then ag_prev=; continue; fi
        case $ag_arg in
            -cache-file|--cache-file|--cache-fil|--cache-fi|--cache-f)
                ag_prev=cache_file ;;
            --cache-|--cache|--cach|--cac|--ca|--c)
                ag_prev=cache_file ;;
            -cache-file=*|--cache-file=*|--cache-fil=*|--cache-fi=*)
                ;;
            --cache-f=*|--cache-=*|--cache=*|--cach=*|--cac=*|--ca=*)
                ;;
            --c=*|--config-cache|-C)
                ;;

            -srcdir|--srcdir|--srcdi|--srcd|--src|--sr)
                ag_prev=srcdir ;;
            -srcdir=*|--srcdir=*|--srcdi=*|--srcd=*|--src=*|--sr=*)
                ;;

            -prefix|--prefix|--prefi|--pref|--pre|--pr|--p)
                ag_prev=prefix ;;
            -prefix=*|--prefix=*|--prefi=*|--pref=*|--pre=*|--pr=*|--p=*)
                ;;

            --disable-option-checking)
                ;;

            --with-templates)
                ag_prev=with_templates ;;
            --with-templates=*)
                ;;

            PKG_CONFIG_PATH=*)
                ;;

            *)
                case $ag_arg in
                    *\'*)
                        ag_arg=`AS_ECHO(["$ag_arg"]) | sed "s/'/'\\\\\\\\''/g"` ;;
                esac
                AS_VAR_APPEND([ag_sub_configure_args], [" '$ag_arg'"]) ;;
        esac
    done

    # use parent cache file
    case $cache_file in
        /*) ag_arg=$cache_file;;
        *) ag_arg=$abs_top_build_dir/$cache_file;;
    esac
    ag_sub_configure_args="$ag_sub_configure_args '--cache-file=$ag_arg'"

    # configure in-place
    ag_sub_configure_args="$ag_sub_configure_args '--srcdir=.'"

    # always append PKG_CONFIG_PATH
    ag_arg="$ac_pwd${PKG_CONFIG_PATH:+:}$PKG_CONFIG_PATH"
    for d in $AG_TEMPLATES_SUBDIRS; do
      ag_arg="$ac_pwd/$d:$ag_arg"
    done
    ag_sub_configure_args="$ag_sub_configure_args 'PKG_CONFIG_PATH=$ag_arg'"

    # always prepend --prefix to ensure using the same prefix in subdir
    # configurations.
    ag_arg="--prefix=$prefix"
    case $ag_arg in
        *\'*) ag_arg=`AS_ECHO(["$ag_arg"]) | sed "s/'/'\\\\\\\\''/g"` ;;
    esac
    ag_sub_configure_args="'$ag_arg' $ag_sub_configure_args"

    # always prepend --disable-option-checking to silence warnings, since
    # different subdirs can have different --enable and --with options.
    ag_sub_configure_args="'--disable-option-checking' $ag_sub_configure_args"
])
