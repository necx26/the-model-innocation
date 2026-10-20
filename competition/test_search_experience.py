import math
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros_ws'))
from search_experience import SearchExperience


class Grid:
    free=np.ones((41,41),dtype=bool)
    def cell(self,p):return round(p[1]*10),round(p[0]*10)
    def xy(self,p):return p[1]/10,p[0]/10
    def inside(self,p):return 0<=p[0]<41 and 0<=p[1]<41
    def safe(self,p):return self.inside(self.cell(p))
    def line_free(self,a,b):return True
    def distances(self,p):return p
    def distance(self,a,b):return math.dist(a[:2],b[:2])


class ExperienceTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.root=Path(self.directory.name)
        (self.root/'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
        (self.root/'map.yaml').write_text('image: map.pgm\n')
        self.experience=SearchExperience(Grid(),self.root/'map.yaml',self.root/'data','test')
    def tearDown(self):self.directory.cleanup()
    def target(self,**extra):return dict(color='red',x=2.,y=2.,hits=5,status='available',**extra)
    def test_unconfirmed_and_unreachable_tracks_do_not_train(self):
        self.experience.learn([dict(self.target(),hits=1),dict(self.target(),status='unreachable')],'round1')
        self.assertIsNone(self.experience.observation((0.,0.,0.),'red',[],(1.,1.)))
    def test_transient_two_frame_tracks_do_not_become_permanent_hotspots(self):
        self.experience.learn([dict(self.target(),hits=2)],'transient')
        self.assertEqual(float(self.experience.heat.sum()),0.)
        self.experience.learn([dict(self.target(),hits=2,status='delivered')],'verified')
        self.assertGreater(float(self.experience.heat.sum()),0.)
    def test_legacy_prior_without_training_policy_is_not_silently_reused(self):
        self.experience.path.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(self.experience.path,heat=np.ones_like(self.experience.heat),runs='["old"]')
        loaded=SearchExperience(Grid(),self.root/'map.yaml',self.root/'data','test')
        self.assertEqual(float(loaded.heat.sum()),0.)
        self.assertEqual(loaded.runs,[])
    def test_prior_only_proposes_an_observation_and_requires_fresh_runtime_detection(self):
        self.experience.learn([self.target()],'round1')
        proposal=self.experience.observation((0.,0.,0.),'red',[],(1.,1.))
        self.assertAlmostEqual(math.dist(proposal['pose'][:2],proposal['centre']),.95)
        self.assertNotIn('target',proposal)
        self.assertNotIn('model',proposal)
        self.assertNotIn('id',proposal)
        self.assertIsNone(self.experience.observation((0.,0.,0.),'red',[proposal['centre']],(1.,1.)))
    def test_report_import_is_idempotent_and_saved_map_reloads(self):
        self.experience.learn([self.target()],'round1');expected=self.experience.heat.copy()
        self.experience.learn([self.target()],'round1')
        np.testing.assert_array_equal(self.experience.heat,expected)
        loaded=SearchExperience(Grid(),self.root/'map.yaml',self.root/'data','test')
        np.testing.assert_array_equal(loaded.heat,expected)
    def test_map_change_does_not_reuse_other_map_landmarks(self):
        self.experience.learn([self.target()],'round1')
        (self.root/'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        loaded=SearchExperience(Grid(),self.root/'map.yaml',self.root/'data','test')
        self.assertEqual(loaded.runs,[])
        self.assertEqual(float(loaded.heat.sum()),0.)


if __name__=='__main__':unittest.main()
