import rospy

class Brain:
	_mood = "sleeping"
	def __init__(self):
		if rospy.has_param("ernest/mood"):
			print "load mood from config"
			self._mood = rospy.get_param("ernest/mood")
		else:
			print "no param :'("
	def set_mood(self, newmood):
		self._mood = newmood
		rospy.set_param("ernest/mood", newmood)
	mood = property(lambda self : self._mood)
