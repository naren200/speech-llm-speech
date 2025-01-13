import sys
import json
import re
from nltk.sentiment.vader import SentimentIntensityAnalyzer

def clean_text(text):
    # Remove special characters and normalize whitespace
    text = re.sub(r'[^\w\s.,!?-]', ' ', text)
    text = ' '.join(text.split())
    return text

def analyze_sentiment(text):
    # Clean the input text
    text = clean_text(text)
    
    # Initialize VADER analyzer
    analyzer = SentimentIntensityAnalyzer()
    
    # Get sentiment scores
    scores = analyzer.polarity_scores(text)
    
    return {
        "polarity": scores,
        "subjectivity": scores['compound'],  # Using compound score as subjectivity
        "compound": scores['compound']
    }

if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            print(json.dumps({"error": "No text provided"}))
            sys.exit(1)
            
        text = sys.argv[1]
        result = analyze_sentiment(text)
        print(json.dumps(result))
    except Exception as e:
        print(json.dumps({
            "polarity": 0.0,
            "subjectivity": 0.0,
            "compound": 0.0
        }))
