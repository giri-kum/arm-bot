ó
wqZc           @   s  d  d l  Z  d  d l Z d  d l Z d  d l Td  d l m Z m Z m Z d  d l	 m
 Z
 d  d l m Z d  d l m Z d  d l m Z d d	 Z d	 d Z d
 Z d Z d Z d Z d a d a d a d a d Z d e j f d     YZ d   Z e d k re   n  d S(   iÿÿÿÿN(   t   *(   t   QtGuit   QtCoret   Qt(   t   Ui_MainWindow(   t   Rexarm(   t   Video(   t   Statemachinegz üú!	@g     f@i6  i¶  i   iþ  g        g     V@t   Guic           B   s  e  Z d  Z d d  Z d   Z d   Z d   Z d   Z d   Z	 d   Z
 d   Z d	   Z d
   Z d   Z d   Z d   Z d   Z d   Z d   Z d   Z d   Z d   Z e d  Z d   Z d   Z d   Z d   Z d   Z d   Z d   Z d   Z  d   Z! RS(   sj    
    Main GUI Class
    contains the main function and interfaces between 
    the GUI and functions
    c         C   sk  t  j j |  |  t   |  _ |  j j |   t   |  _ t   |  _	 t
   |  _ t j d d g  |  _ t  j j |  t  |  j j |  j |  j  t j |   |  _ |  j j j |  j  |  j j d  t j |   |  _ |  j j j |  j j  |  j j   |  j j j j |  j  |  j j j j |  j  |  j j  j j |  j!  |  j j" j j |  j#  |  j j$ j j |  j%  |  j j& j j |  j'  |  j j( j j |  j)  |  j j* j j |  j)  |  j j+ d k r
|  j j$ j, t  |  j j& j, t  n& |  j j$ j, t-  |  j j& j, t-  |  j.   |  j/   |  j j0 j1 d  |  j j0 j2 j |  j3  |  j j4 j1 d  |  j j4 j2 j |  j5  |  j j6 j1 d  |  j j6 j2 j |  j7  |  j j8 j1 d  |  j j8 j2 j |  j9  |  j j: j1 d  |  j j: j2 j |  j;  |  j j< j1 d	  |  j j< j2 j |  j=  |  j j> j1 d
  |  j j> j2 j |  j?  |  j j> j, t-  |  j j@ j1 d  |  j j@ j2 j |  jA  |  j jB j1 d  |  j jB j2 j |  jC  |  j jD j1 d  |  j jD j2 j |  j/  |  j jE j1 d  |  j jE j2 j |  jF  |  j jG j1 d  |  j jG j2 j |  jH  d  S(   Ni    i   i   s   Configure Servoss   Depth and RGB Calibrations   Extrinsic Calibrations   Block Detectort   Repeatt   Teachs   Smooth Repeats
   Pick Blocks   Place Blockt   Resets   Enter Competition Modes   Emergency Stop!(I   R   t   QWidgett   __init__R   t   uit   setupUiR   t   rexR   t   videoR   t   statemachinet   npt   float32t
   last_clickt   setMouseTrackingt   Truet   setmeR   t   QTimert   _timert   timeoutt   connectt
   update_guit   startt   _timer2t   get_feedbackt   sldrBaset   valueChangedt   sliderBaseChanget   sldrShouldert   sliderShoulderChanget	   sldrElbowt   sliderElbowChanget	   sldrWristt   sliderWristChanget	   sldrGrip1t   sliderGrip1Changet	   sldrGrip2t   sliderGrip2Changet   sldrMaxTorquet   sliderOtherChanget	   sldrSpeedt
   num_jointst
   setEnabledt   Falset   sliderChanget   resett   btnUser1t   setTextt   clickedt   btn1t   btnUser2t   btn2t   btnUser3t   btn3t   btnUser4t   btn4t   btnUser5t   btn5t   btnUser6t   teacht   btnUser7t   srepeatt   btnUser8t   pickt   btnUser9t   placet	   btnUser10t	   btnUser11t   competitiont	   btnUser12t   estop(   t   selft   parent(    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR   #   sn    

c         C   s   |  j  j |  j  d  S(   N(   R   RN   R   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyRN      s    c         C   s]   | d k s | d k  r! d } n8 | d k r: | d } n | d k  rS d | } n | } | S(   Nih  iþÿÿi    i´   iLÿÿÿ(    (   RO   t   qt   new_q(    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyt
   trim_angle   s    	c         C   sS  |  j  j   |  j j d k rz y |  j j   |  j j   Wqz t k
 rv d |  j _ |  j j   |  j j   qz Xn  |  j	 j
 j   r« |  j	 j j |  j j    n  |  j	 j j   rÜ |  j	 j j |  j j    n  |  j	 j j t d |  j j d t   |  j	 j j t d |  j j d t   |  j	 j j t d |  j j d t   |  j	 j j t d |  j j d t   |  j	 j j t |  j	 j j     |  j	 j j t |  j	 j j     t |  j j d t |  j j d t |  j j d t |  j j d t  } |  j	 j  j t d t! | d d    |  j	 j" j t d t! | d d    |  j	 j# j t d t! | d d    |  j	 j$ j t d t! | d d    t% j& j' |  t% j( j)    j*   } t% j& j' |  t% j( j)    j+   } | t, k  s?| t- k s?| t. k  s?| t/ k rh|  j	 j0 j d  |  j	 j1 j d  n| t, } | t. } |  j j2 | | } |  j	 j0 j d | | | f  |  j j3 d k r |  j j4 } t5 j6 | | d g  } t5 j7 | |  } |  j	 j1 j d	 | d | d f  n×|  j j8 d k rät5 j6 | | g g g  } t5 j9 | | d
 g  }	 t5 j9 d d d g d d d g d d d
 g g  |  j _: t5 j7 |  j j: |	  }
 t; |
 d  } t; |
 d  } |  j j2 | | } d } xd t< t= |  j j>   D]J } | |  j j? | k r	| |  j j@ | k r	d |  j j> | } Pq	q	W| tA jB | |  j jC |  j jD  } t5 jE t5 jF | | d
 g   } t5 j7 |  j jG |  } |  j	 j1 j d | d | d | d f  n |  j	 j1 j d  |  j jH d k r-|  j	 jI j d |  j jJ d  n  |  j	 jI j |  j  jK d tL   d S(   sM    
        update_gui function
        Continuously called by timer1 
        i   i    s   %.2fi   i   s   %2fs   (-,-,-)s   (%.0f,%.0f,%.0f)s   (%.0f,%.0f,0)g      ð?g¨^Ð¥,ñ?g.>=ñ5l?gùPú¿g½ÈIiÒc¿gÀh 4ñ?g.üÃwç:Àg        iÿÿÿÿi­  s   Playing Back - Waypoint %dt   combinedN(M   R   t   timerCallbackR   t   kinectConnectedt   captureVideoFramet   captureDepthFramet	   TypeErrort   loadVideoFramet   loadDepthFrameR   t
   radioVideot	   isCheckedt
   videoFramet	   setPixmapt   convertFramet
   radioDeptht   convertDepthFramet   rdoutBaseJCR7   t   strR   t   joint_angles_fbt   R2Dt   rdoutShoulderJCt   rdoutElbowJCt   rdoutWristJCt
   rdoutGrip1R*   t   valuet
   rdoutGrip2R,   t   forwardKinematicst   rdoutXt   roundt   rdoutYt   rdoutZt   rdoutTR   R   t   mapFromGlobalt   QCursort   post   xt   yt   MIN_Xt   MAX_Xt   MIN_Yt   MAX_Yt   rdoutMousePixelst   rdoutMouseWorldt   currentDepthFramet   cal_flagt
   aff_matrixR   R   t   dott   extrinsic_cal_flagt   arrayt   rgb2dep_aff_matrixt   intt   ranget   lent	   levels_mmt   levels_upper_dt   levels_lower_dt   cv2t   undistortPointst	   intrinsict   distortion_arrayt	   transposet   appendt	   extrinsict   plan_statust   rdoutStatust
   wpt_numbert   getmestatusR   (   RO   t   endCoordRv   Rw   t   zt   Mt   vt   rwt	   xy_in_rgbt   xy_in_rgb_homogenoust	   xy_in_dept   x_dept   y_dept   dt   Zt   idxt   camera_coordt   camera_coord_homogenoust   world_coord_homogenous(    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR      sx    ++++%%M****$$0

 (9,%!/c         C   sy  |  j  j j t |  j  j j     |  j  j j t |  j  j j    d  |  j  j j   d |  j _	 |  j  j
 j   d g |  j j |  j _ |  j  j j   t |  j j d <|  j  j j   t |  j j d <|  j  j j   t |  j j d <|  j  j j   t |  j j d <|  j j d k rh|  j  j j   t |  j j d <|  j  j j   t |  j j d	 <n  |  j j   d
 S(   s    
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position
        t   %g      Y@i    i   i   i   i   i   i   N(   R   t	   rdoutBaseR7   Rd   R!   Rk   t	   rdoutTorqR.   R   t   torque_multiplierR0   R1   t   speed_multipliert   D2Rt   joint_anglesR$   R&   R(   t   cmd_publish(   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR4     s    %))     #c         C   sa  |  j  j j t |  j  j j     |  j  j j t |  j  j j     |  j  j j t |  j  j	 j     |  j  j
 j t |  j  j j     |  j  j j t |  j  j j     |  j  j j t |  j  j j     |  j  j j t |  j  j j    d  |  j  j j t |  j  j j    d  |  j  j j   d |  j _ |  j  j j   d g |  j j |  j _ |  j  j j   t |  j j d <|  j  j j   t |  j j d <|  j  j	 j   t |  j j d <|  j  j j   t |  j j d <t |  j j  d k rP|  j  j j   t |  j j d <|  j  j j   t |  j j d	 <n  |  j j   d
 S(   s    
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position
        R¦   g      Y@i    i   i   i   i   i   i   N(   R   R§   R7   Rd   R!   Rk   t   rdoutShoulderR$   t
   rdoutElbowR&   t
   rdoutWristR(   Rj   R*   Rl   R,   R¨   R.   t
   rdoutSpeedR0   R   R©   R1   Rª   R«   R¬   R   R­   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyt   sliderinitChange   s$    %%%%%%)))     #c         C   s¨   |  j  j j t |  j  j j    d  |  j  j j t |  j  j j    d  |  j  j j   d |  j _	 |  j  j j   d g |  j j
 |  j _ |  j j   d  S(   NR¦   g      Y@(   R   R¨   R7   Rd   R.   Rk   R±   R0   R   R©   R1   Rª   R­   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR/   >  s
    )))c         C   sV   |  j  j j t |  j  j j     |  j  j j   t |  j j d <|  j j	   d  S(   Ni    (
   R   R§   R7   Rd   R!   Rk   R«   R   R¬   R­   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR#   E  s    % c         C   sV   |  j  j j t |  j  j j     |  j  j j   t |  j j d <|  j j	   d  S(   Ni   (
   R   R®   R7   Rd   R$   Rk   R«   R   R¬   R­   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR%   J  s    % c         C   sV   |  j  j j t |  j  j j     |  j  j j   t |  j j d <|  j j	   d  S(   Ni   (
   R   R¯   R7   Rd   R&   Rk   R«   R   R¬   R­   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR'   O  s    % c         C   sV   |  j  j j t |  j  j j     |  j  j j   t |  j j d <|  j j	   d  S(   Ni   (
   R   R°   R7   Rd   R(   Rk   R«   R   R¬   R­   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR)   T  s    % c         C   sV   |  j  j j t |  j  j j     |  j  j j   t |  j j d <|  j j	   d  S(   Ni   (
   R   Rj   R7   Rd   R*   Rk   R«   R   R¬   R­   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR+   Z  s    % c         C   sV   |  j  j j t |  j  j j     |  j  j j   t |  j j d <|  j j	   d  S(   Ni   (
   R   Rl   R7   Rd   R,   Rk   R«   R   R¬   R­   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR-   _  s    % c         C   sM  | j    } | j   } | t k  sH | t k sH | t k  sH | t k rL d S| t |  j d <| t |  j d <|  j j d k rP| t | t g |  j j	 |  j j
 <|  j j
 d 7_
 |  j j j d |  j j
 |  j j
 d f  |  j j
 |  j j k rPd |  j _ d |  j _
 t j |  j j	 |  j j  |  j _ |  j j j d  qPn  |  j j d k r|  j j
 |  j j k  r½| t |  j j |  j j
 d <| t |  j j |  j j
 d d <n | t | t d d d d g |  j j d |  j j
 |  j j <d d d | t | t d g |  j j d |  j j
 |  j j d <|  j j
 d 7_
 |  j j
 GH|  j j
 |  j j k rd |  j _ d |  j _
 |  j j } t j |  } t j | |  } t j j |  } t j | |  } t j |  j j  }	 t j | |	  }
 t j |
 d  } t j d d d g g  } t j | | f d d |  j _  t j j |  j j   |  j _! |  j j! GHqn  |  j j" d k rI| t | t g |  j j# |  j j
 <|  j j
 d 7_
 |  j j
 d k rId |  j _
 t j$ |  j j# d g |  j j# d g |  j j# d g g  } d	 t j% | |  j j& |  j j'  } d
 t j( | d d d | d d d | d d d | d d d  } | GH| d k  r¼| d t j) } n d t j) | } t j t j* |  t j+ |  d g t j+ |  d
 t j* |  d g d d d
 g g  } t j |  } t j$ | d d d | d d d d | d d d | d d d d d g  } t j t j d
 t j | |  g   } t j | | f d d } t j | t j$ d d d d g g  f d d |  j _, t j j |  j j,  |  j _- |  j j, GHd |  j _" qIn  d S(   sQ    
        Function used to record mouse click positions for calibration 
        Ni    i   sL   Affine Calibration: Click Point %d is recorded, please select Click Point %di   s   Waiting for inputi   t   axisg     h@g      ð¿g      à?g        i­  g      ð?(   i   i   (.   Rv   Rw   Rx   Ry   Rz   R{   R   R   R   t   mouse_coordt   mouse_click_idR   R   R7   t
   cal_pointsR   t   getAffineTransformt
   real_coordR   t   dep2rgb_aff_flagt   rgb_pts_numt	   rgb_coordt	   dep_coordt   dep_pts_numR   R   R   t   linalgt   invt   reshapeR   t   concatenatet   dep2rgb_aff_matrixR   R   t   extrinsic_cal_pixel_coordR   R   R   R   t   arctan2t   pit   cost   sinR   t   inv_extrinsic(   RO   t   QMouseEventRv   Rw   t   At   A_transt   A_ATt   A_AT_invt   A_allt   bt   tmp_dep2rgb_aff_arrayt   upper_dep2rgb_aff_matrixt   lower_dep2rgb_aff_matrixt	   tmp_coordR£   t   rot_radt   z_rott   R_Tt   Rt   Tt   tmp(    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyt   mousePressEventf  s|    0 $		%>B!$?%N[X+9c         C   s#   d |  j  _ |  j j j d  d  S(   Ni   sB   Start camera calibration by clicking mouse to select Click Point 1(   R   R¹   R   R   R7   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyt   deprgb_caliÕ  s    c         C   s  d } d } d } d } t  j d | d d g d d | d g d d d g g  } t  j d d | g d d | g d d d g g  } t  j | |  |  j _ t  j | d | g d | | g d d d g g  |  j _ t  j d d	 d
 d d g  |  j _ d |  j _ d  S(   NgKOo@gsqm£s@gÚ.<H*t@g¸Dùn@i   g        g      ð?gl{áìÛkÏ?g]Æ}FJ|ê¿g.öf×XQ?g¸ÆK¼h¿g·PÇ±cñ?(   R   R   R   R   t   inv_intrinsicR   R   R   (   RO   t   alphat   u0t   betat   v0t   matrix1t   matrix2(    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyt   extrinsic_caliÚ  s    ;59$c         C   s   |  j  j |  j |  j  d  S(   N(   R   RC   R   R   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyRC   é  s    c         C   sW   |  j  j j   d k r7 |  j j |  j  |  j d  n |  j j |  j  |  j t  d  S(   NR	   t   Initialising(   R   RD   t   textR   t   initR   t   repeatR3   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyRç   ì  s    c         C   s    |  j  j |  j |  j d  d  S(   Nt   initialising(   R   Ræ   R   R   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyRE   ò  s    c         C   s,  |  j  j j   d |  j _ |  j  j j   d g |  j j |  j _ d |  j j d <d |  j j d <d |  j j d <d |  j j d <t	 |  j j  d k rÐ | t
 k rÐ d |  j j d <d	 t |  j j d
 <nK t	 |  j j  d k r| t k rd |  j j d <d t |  j j d
 <n  |  j j   d  S(   Ng      Y@g        i    i   i   i   i   i   i_   i   i¡ÿÿÿ(   R   R.   Rk   R   R©   R0   R1   Rª   R¬   R   R3   R«   R   R­   (   RO   t   hold(    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR5   õ  s    )$$c         C   sO  |  j  d  \ } } } } | | | | g GHd | | | | g \ a a a a | d | d | d d t g } d GH| GHt | d | d | d | d	  } t |  j | d  d  | d <t |  j | d  d  | d <t |  j | d  d  | d <t |  j | d	  d  | d	 <|  j	 j
 | | |  |  j	 j d
 d d  d  S(   Nt   blueiÿÿÿÿi
   i,   s	   endcoord:i    i   i   i   t   testingt   picking(   t   get_color_block_world_coordt   putxt   putyt   putzt   putanglet   gripper_orientationt   inverseKinematicsRo   RS   R   t   setqt   setmystatus(   RO   t   blockxt   blockyt   blockzt   angleR   t   angles(    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyRG     s    ""%    c         C   s  t  t t t g GHt  d t d t d d t g } t | d | d | d | d  } t |  j | d  d  | d <t |  j | d  d  | d <t |  j | d  d  | d <t |  j | d  d  | d <|  j j	 | | |  |  j j
 d d d  d  S(	   Ni
   i   i    i   i   i   Rë   t   placing(   Rî   Rï   Rð   Rñ   Rò   Ró   Ro   RS   R   Rô   Rõ   (   RO   R   Rú   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyRI     s    "%    c         C   s  |  j  j   |  j  j |  \ \ } } } t j | | g g g  } t j | | d g  } t j |  j  j |  } t | d  } t | d  }	 |  j  j	 |	 | }
 d } xd t
 t |  j  j   D]J } |
 |  j  j | k rÉ |
 |  j  j | k rÉ d |  j  j | } PqÉ qÉ W| t j | |  j  j |  j  j  } t j t j | | d g   } t j |  j  j |  } | d | d | d | f S(   Ng      ð?i    i   iÿÿÿÿi­  i   (   R   t   blockDetectort   color_detectionR   R   R   R   R   R   R~   R   R   R   R   R   R   R   R   R   R   R   R   (   RO   t   colort
   blockx_rgbt
   blocky_rgbRù   R   R   R   R   R   R    R¡   R¢   R£   R¤   R¥   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyRí   -  s"    ,%!c         C   sW  |  j  j j   d k r³ |  j  j j d  |  j  j j d  |  j  j j d  |  j  j j d  |  j  j j d  |  j  j j d  |  j  j	 j d  |  j  j j
 t  n  |  j  j j   d k rS|  j  j j d  |  j  j j d	  |  j  j j d
  |  j  j j d  |  j  j j d  |  j  j j d  |  j  j	 j d  n  d  S(   Ns   Enter Competition Modes   Enter Testing Modet   Draws   Competition 1s   Competition 2s   Competition 3s   Competition 4s   Competition 5s   Smooth Repeats   Configure Servoss   Depth and RGB Calibrations   Extrinsic Calibrations   Block DetectionR	   (   R   RK   Rå   R7   RD   R6   R:   R<   R>   R@   R2   R   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyRL   C  s"    c         C   sm   |  j  j j   d k r( |  j j   nA |  j  j j   d k ri |  j j d  |  j j d d d  n  d  S(   Ns   Configure Servoss   Competition 1Rê   Rì   (   R   R6   Rå   R   t   cfg_publish_defaultR   t	   getanglesRõ   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR9   W  s
    c         C   sD   |  j  j j   d k r% |  j   n |  j  j j   d k r@ n  d  S(   Ns   Depth and RGB Calibrations   Competition 2(   R   R:   Rå   RÛ   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR;   ^  s    c         C   sD   |  j  j j   d k r% |  j   n |  j  j j   d k r@ n  d  S(   Ns   Extrinsic Calibrations   Competition 3(   R   R<   Rå   Rã   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR=   d  s    c         C   sG   |  j  j j   d k r( |  j j   n |  j  j j   d k rC n  d  S(   Ns   Block Detectors   Competition 4(   R   R>   Rå   R   Rü   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR?   j  s    c         C   sD   |  j  j j   d k r% |  j   n |  j  j j   d k r@ n  d  S(   NR	   s   Competition 5(   R   R@   Rå   Rç   (   RO   (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyRA   p  s    N("   t   __name__t
   __module__t   __doc__t   NoneR   RN   RS   R   R4   R²   R/   R#   R%   R'   R)   R+   R-   RÚ   RÛ   Rã   RC   Rç   RE   R3   R5   RG   RI   Rí   RL   R9   R;   R=   R?   RA   (    (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyR      s<   k			p										o													c          C   s<   t  j t j  }  t   } | j   t j |  j    d  S(   N(   R   t   QApplicationt   syst   argvR   t   showt   exitt   exec_(   t   appt
   app_window(    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyt   mainx  s    	
t   __main__(    R	  R   t   numpyR   t
   kinematicst   PyQt4R   R   R   R   R   t   rexarmR   R   R   R   R   R«   Rf   Rx   Ry   Rz   R{   Rî   Rï   Rð   Rñ   Rò   t   QMainWindowR   R  R  (    (    (    sH   /home/student/rob-550-lab1_kig_integrating/armlab-W18/control_station.pyt   <module>   s2   


ÿ ÿ ]	