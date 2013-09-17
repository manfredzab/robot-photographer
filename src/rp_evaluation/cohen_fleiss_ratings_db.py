#!/usr/bin/python
import MySQLdb
import cohen_fleiss
from collections import defaultdict

# Connect to the rating DB
db = MySQLdb.connect(host =   "HOST NAME",
                     user =   "USER NAME",
                     passwd = "PASSWORD",
                     db =     "DB NAME")

cursor = db.cursor() 

# Get unique rater IDs
cursor.execute("SELECT DISTINCT SessionID FROM Ratings")
ids = [item[0] for item in cursor.fetchall()]

# Get all ratings from all raters
ratings = defaultdict(int)
cursor.execute("SELECT SessionID, Photo, Rating FROM Ratings")
for item in cursor.fetchall():
	session_id = item[0]
	photo_id = item[1]
	rating = item[2]

	ratings[(session_id, photo_id)] = rating

# Calculate Cohen's kappa for all raters
for id1 in ids:
	for id2 in ids:
		rating_list1 = []
		rating_list2 = []

		distance = 0
		for photo_number in range(1, 104):
			photo_id = "img/%d.JPG" % photo_number
			rating_list1.append(ratings[(id1, photo_id)])
			rating_list2.append(ratings[(id2, photo_id)])

			distance += abs(ratings[(id1, photo_id)] - ratings[(id2, photo_id)])

		#quadratic_weighted_kappa = cohen_fleiss.ratings(rating_list1, rating_list2, 5)
		#print ("%.4f" % quadratic_weighted_kappa),"\t",

		print ("%d" % distance),"\t",
	print