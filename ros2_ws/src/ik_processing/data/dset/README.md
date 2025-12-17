# Dataset Information

## Strucutre

1. Level: interaction vs baseline
2. Level: move, encoded in table numeral scheme
3. Level: participants-group number
4. Level: repetitions (only for baselines multiple repetitions)
 
## Data

- each dataset file is a dat file, tab characters used as delimiters
- data is organized column-wise, a new line is indicated by a linebreak
	- baselines: rows x cols = 4 x n, where the four rows are: time, position in x, pos in y, pos in z
	- interactions: rows x cols = 7 x n, -- time, xAgent1, yAgent1, zAgent1, xAgent2, yAgent2, zAgent2

- coordinate system, such that the table surface + can height = z = 0

## participant information

- trial number: left vs right hand, gender

## Tablepoints

- each row is a 3D table point corresponding to it's numeral. z=0 is height of the can. 

