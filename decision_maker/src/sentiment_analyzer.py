import sys
import json
from textblob import TextBlob
from vaderSentiment.vaderSentiment import SentimentIntensityAnalyzer

def analyze_sentiment(text):
    # TextBlob analysis
    blob = TextBlob(text)
    
    # VADER analysis
    analyzer = SentimentIntensityAnalyzer()
    vader_scores = analyzer.polarity_scores(text)
    
    return {
        "polarity": blob.sentiment.polarity,
        "subjectivity": blob.sentiment.subjectivity,
        "compound": vader_scores['compound']
    }

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(json.dumps({"error": "No text provided"}))
        sys.exit(1)
        
    text = sys.argv[1]
    result = analyze_sentiment(text)
    print(json.dumps(result))
