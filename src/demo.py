import rospy
import baxter_interface

rospy.init_node('Game_demo')


limbR = baxter_interface.Limb('right')
limbL = baxter_interface.Limb('left')

reset = {'right_s0': 0.9157865293212891, 'right_s1': -0.515417544140625, 'right_w0': -0.3915485956604004, 'right_w1': 1.3011992018371583, 'right_w2': 0.4157087930419922, 'right_e0': 0.31101460438842776, 'right_e1': 0.8567282690551759}

ready = {'left_w0': 0.19059711267700197, 'left_w1': 2.093883773071289, 'left_w2': -0.30564567163696293, 'left_e0': -0.4272136489379883, 'left_e1': 0.25272333451538087, 'left_s0': -0.8417719563903809, 'left_s1': -0.9387962411132813}

posA1U = {'right_s0': 1.498699228051758, 'right_s1': -0.05752427947998047, 'right_w0': 0.6331505694763184, 'right_w1': 1.2302525904785158, 'right_w2': 0.3558835423828125, 'right_e0': -0.7251894166442872, 'right_e1': 0.3428447057006836}


posA1D = {'right_s0': 1.275888518865967, 'right_s1': -0.13805827075195312, 'right_w0': -0.23968449783325196, 'right_w1': 0.9085001205871582, 'right_w2': 0.43219908649291994, 'right_e0': 0.12885438603515625, 'right_e1': 0.6216457135803223}


posA2U = {'right_s0': 1.2916118219238282, 'right_s1': -0.39269908125, 'right_w0': -0.2427524594055176, 'right_w1': 0.9123350725524902, 'right_w2': 0.4747670533081055, 'right_e0': 0.12885438603515625, 'right_e1': 0.998621491772461}

posA2D = {'right_s0': 1.2858593939758303, 'right_s1': -0.3631699511169434, 'right_w0': -0.2174417764343262, 'right_w1': 0.8130098166503906, 'right_w2': 0.5146505537475586, 'right_e0': 0.11926700612182618, 'right_e1': 1.0618981992004395}


posA3U = {'right_s0': 1.5389662236877442, 'right_s1': -0.4007524803771973, 'right_w0': 0.4460049135681153, 'right_w1': 1.180014719732666, 'right_w2': 0.20440293975219728, 'right_e0': -0.4038204419494629, 'right_e1': 0.9901845974487306}

posA3D = {'right_s0': 1.3169225048950197, 'right_s1': -0.44101947601318364, 'right_w0': -0.04793689956665039, 'right_w1': 0.7600874795288086, 'right_w2': 0.40918937470092775, 'right_e0': 0.0724805921447754, 'right_e1': 1.2360050184265137}


posB1U = {'right_s0': 1.3203739616638184, 'right_s1': -0.3044951860473633, 'right_w0': 0.2880048925964356, 'right_w1': 1.2160632682067871, 'right_w2': 0.14457768909301758, 'right_e0': -0.3520485904174805, 'right_e1': 0.5848301747131348}


posB1D = {'right_s0': 1.1359127721313478, 'right_s1': -0.16030099215087892, 'right_w0': -0.28532042622070314, 'right_w1': 0.9694758568359375, 'right_w2': 0.28455343582763676, 'right_e0': 0.18292720874633792, 'right_e1': 0.6534758148925782}


prePick0 = {'right_s0': 0.9315098323791504, 'right_s1': -0.6599952332336426, 'right_w0': -0.3275048978393555, 'right_w1': 0.9552865345642091, 'right_w2': 0.28340295023803713, 'right_e0': 0.22741265154418946, 'right_e1': 1.238305989605713}

prePick1 = {'right_s0': 0.9387962411132813, 'right_s1': -0.6615292140197754, 'right_w0': -0.3271214026428223, 'right_w1': 0.8018884559509278, 'right_w2': 0.3087136332092285, 'right_e0': 0.20478643494873047, 'right_e1': 1.3299613415771485}

prePick2 = {'right_s0': 0.9767622655700684, 'right_s1': -0.6212622183837891, 'right_w0': -0.32827188823242187, 'right_w1': 0.8018884559509278, 'right_w2': 0.3501311144348145, 'right_e0': 0.16030099215087892, 'right_e1': 1.2996652210510256}


posStack0 = {'right_s0': 0.9572040105468751, 'right_s1': -0.6224127039733887, 'right_w0': -0.3390097537353516, 'right_w1': 0.8241311773498535, 'right_w2': 0.3773592733886719, 'right_e0': 0.19903400700073243, 'right_e1': 1.3054176489990235}

posStack1 = {'right_s0': 0.9173205101074219, 'right_s1': -0.5687233764587403, 'right_w0': -0.3420777153076172, 'right_w1': 0.7462816524536133, 'right_w2': 0.3785097589782715, 'right_e0': 0.22741265154418946, 'right_e1': 1.319606971270752}


posStack2 = {'right_s0': 0.9951700350036622, 'right_s1': -0.4084223843078614, 'right_w0': -0.3347913065734863, 'right_w1': 0.8954612839050293, 'right_w2': 0.2880048925964356, 'right_e0': 0.1449611842895508, 'right_e1': 1.1075341275878907}

posStack3 = {'right_s0': 1.0185632419921875, 'right_s1': -0.3213689746948242, 'right_w0': -0.07631554411010742, 'right_w1': 0.8030389415405274, 'right_w2': 0.09587379913330078, 'right_e0': 0.01457281746826172, 'right_e1': 1.0595972280212402}




right_gripper = baxter_interface.Gripper('right')

def prePick():
	"readies the claw for picking a block"
	limbR.move_to_joint_positions(prePick0)
	limbR.move_to_joint_positions(prePick1)
	limbR.move_to_joint_positions(prePick2)
	return


#Set up the robot in the environment
right_gripper.open()
limbR.move_to_joint_positions(reset)
limbL.move_to_joint_positions(ready)

#Get image from right arm camera, detect block type

prePick()
limbR.move_to_joint_positions(posStack0)
right_gripper.close()
limbR.move_to_joint_positions(reset)
limbR.move_to_joint_positions(posA3U)
limbR.move_to_joint_positions(posA3D)
right_gripper.open()
limbR.move_to_joint_positions(posA3U)
limbR.move_to_joint_positions(reset)

prePick()
limbR.move_to_joint_positions(posStack0)
limbR.move_to_joint_positions(posStack1)
right_gripper.close()
limbR.move_to_joint_positions(reset)
limbR.move_to_joint_positions(posA2U)
limbR.move_to_joint_positions(posA2D)
right_gripper.open()
limbR.move_to_joint_positions(posA2U)
limbR.move_to_joint_positions(reset)

prePick()
limbR.move_to_joint_positions(posStack0)
limbR.move_to_joint_positions(posStack1)
limbR.move_to_joint_positions(posStack2)
right_gripper.close()
limbR.move_to_joint_positions(reset)
limbR.move_to_joint_positions(posA1U)
limbR.move_to_joint_positions(posA1D)
right_gripper.open()
limbR.move_to_joint_positions(posA1U)
limbR.move_to_joint_positions(reset)


prePick()
limbR.move_to_joint_positions(posStack0)
limbR.move_to_joint_positions(posStack1)
limbR.move_to_joint_positions(posStack2)
limbR.move_to_joint_positions(posStack3)
right_gripper.close()
limbR.move_to_joint_positions(reset)
limbR.move_to_joint_positions(posB1U)
limbR.move_to_joint_positions(posB1D)
right_gripper.open()
limbR.move_to_joint_positions(posB1U)
limbR.move_to_joint_positions(reset)


quit()




