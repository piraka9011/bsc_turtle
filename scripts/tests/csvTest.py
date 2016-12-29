import csv

csvName = 'CsvTest.csv'

csvFile = open(csvName, 'w')
csvWriter = csv.writer(csvFile)
csvWriter.writerow(('Name:', 'Time:', 'Distance:', 'Type:'))
