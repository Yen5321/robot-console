import unittest
from tools.analyze_jog_log import analyze


class JogLogTests(unittest.TestCase):
    def records(self, distance=1):
        pose=[100,0,200,0,10,0]
        hold={'time':1.,'pose':pose}
        return [{'event':'sample','time':1+i*.05,'arm':{'pose':[100,distance,200,0,10,0],'last_hold':hold}} for i in range(21)]
    def test_stable_within_limits(self):
        result=analyze(self.records())['stops'][0]
        self.assertEqual(result['result'],'WITHIN_TRIAL_LIMITS')
        self.assertEqual(result['max_translation_mm'],1)
        self.assertEqual(result['stable_samples_confirmed_ms'],100)
    def test_overshoot_fails(self):
        self.assertEqual(analyze(self.records(2.1))['stops'][0]['result'],'EXCEEDS_TRIAL_LIMITS')
    def test_short_log_or_feedback_gap_not_passed(self):
        self.assertEqual(analyze(self.records()[:3])['stops'][0]['result'],'INSUFFICIENT_DATA')
        data=self.records();del data[5:10]
        self.assertEqual(analyze(data)['stops'][0]['result'],'INSUFFICIENT_DATA')
    def test_next_motion_excluded(self):
        data=self.records()+[{'event':'control','time':1.2,'mode':2,'input':[1,0,0,0,0]}]
        self.assertEqual(analyze(data)['stops'][0]['result'],'INSUFFICIENT_DATA')

