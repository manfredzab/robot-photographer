#!/usr/bin/python
from collections import defaultdict

def linear_disagreement_weight(rating1, rating2):
	return abs(rating1 - rating2)


def quadratic_disagreement_weight(rating1, rating2):
	return (rating1 - rating2)**2


def ratings(id1, id2, max_rating):
	# Build an agreement matrix and total ratings per rater
	agreement = defaultdict(int)
	total_ratings1 = defaultdict(int)
	total_ratings2 = defaultdict(int)
	total_ratings = 0

	for (rating1, rating2) in zip(id1, id2):
		agreement[(rating1, rating2)] += 1
		total_ratings1[rating1] += 1
		total_ratings2[rating2] += 1
		total_ratings += 1

	# Calculate the total weighted disagreement
	weighted_disagreement = 0.0
	for (rating1, rating2), rating_count in agreement.iteritems():
		weighted_disagreement += quadratic_disagreement_weight(rating1, rating2) * rating_count

	# Calculate the total expected weighted disagreement
	expected_weighted_disagreement = 0.0
	for i in range(1, max_rating + 1):
		for j in range(1, max_rating + 1):
			expected_weighted_disagreement += float(quadratic_disagreement_weight(i, j) * total_ratings1[i] * total_ratings2[j]) / float(total_ratings)

	# Calculate the Cohen-Fleiss kappa
	return 1.0 - weighted_disagreement / expected_weighted_disagreement