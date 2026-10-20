"""Regressions for the observed small-car noun and v2 map-origin failures."""
import json
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros_ws'))
from question_math import demand_tool
from mission_strategy import crop_occupancy_map
from cube_perception import floor_cube_rejection, strict_color_candidates, confirmed_near_target


class CubePerceptionTests(unittest.TestCase):
    def test_side_clipped_carried_color_is_not_a_floor_target(self):
        hsv=np.zeros((100,120,3),np.uint8);hsv[:,:,2]=150
        hsv[40:58,110:120]=[0,250,160]
        self.assertEqual(strict_color_candidates(cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)),[])

    def test_wood_orange_and_cyan_are_not_target_colors(self):
        for hsv in ([12,200,100],[20,230,150],[95,240,160],[0,120,140]):
            image=np.zeros((100,120,3),np.uint8);image[:,:,2]=150
            image[40:55,40:55]=hsv
            self.assertEqual(strict_color_candidates(cv2.cvtColor(image,cv2.COLOR_HSV2BGR)),[])

    def test_saturated_red_and_blue_cubes_are_retained(self):
        hsv=np.zeros((100,120,3),np.uint8);hsv[:,:,2]=150
        hsv[40:55,20:35]=[0,250,160];hsv[40:55,70:85]=[120,250,130]
        self.assertEqual({d['color'] for d in strict_color_candidates(cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR))},
                         {'red','blue'})

    def test_five_centimeter_cube_on_neutral_floor_is_kept(self):
        image=np.zeros((100,120,3),np.uint8);image[:,:,2]=150
        self.assertIsNone(floor_cube_rejection(image,(40,40,10,10),1.,200.,200.))

    def test_same_size_color_patch_on_wood_is_rejected(self):
        image=np.zeros((100,120,3),np.uint8);image[:]=[15,160,150]
        self.assertIn('墙面',floor_cube_rejection(image,(40,40,10,10),1.,200.,200.))

    def test_large_colored_wall_is_rejected_even_with_floor_below(self):
        image=np.zeros((100,120,3),np.uint8);image[:,:,2]=150
        self.assertIn('尺寸',floor_cube_rejection(image,(10,10,60,60),1.,200.,200.))

    def test_clipped_bottom_cannot_be_grabbed(self):
        image=np.zeros((100,120,3),np.uint8);image[:,:,2]=150
        self.assertIn('地面',floor_cube_rejection(image,(40,90,10,10),1.,200.,200.))

    def test_close_view_refines_only_a_confirmed_matching_target(self):
        target={'id':1,'color':'red','x':1.,'y':2.,'hits':3}
        candidate={'color':'red','reason':'颜色位于墙面或非地面背景',
                   'ground_projection':[1.03,1.98],'box':[20,20,42,23]}
        self.assertEqual(confirmed_near_target(candidate,target),(1.03,1.98))
        self.assertIsNone(confirmed_near_target(dict(candidate,color='blue'),target))
        self.assertIsNone(confirmed_near_target(dict(candidate,ground_projection=[1.4,2.]),target))
        self.assertIsNone(confirmed_near_target(dict(candidate,reason='实际尺寸不符合5厘米方块'),target))
        self.assertIsNone(confirmed_near_target(candidate,dict(target,hits=1)))


class QuestionTests(unittest.TestCase):
    def test_generator_small_car_item_is_not_a_person(self):
        question = ('题目: 每个玩具箱有5个玩具;有小车、积木两种玩具箱;'
                    '小杰想玩4个小车,1个积木;小乐、小欣都想玩5个小车;'
                    '小琪想玩4个小车,1个积木;小浩想玩2个积木。'
                    '设小车玩具箱数量为x,积木玩具箱数量为y,计算x、y分别为多少。')
        result = demand_tool(question)
        self.assertEqual(set(result['demands']), {'小杰','小乐','小欣','小琪','小浩'})
        self.assertEqual(result['totals'], [18,4])
        self.assertEqual((result['x'], result['y']), (4,1))

    def test_comparison_references_and_grouped_people_still_work(self):
        question = ('题目: 每个篮有5个水果;有苹果、橘子两种篮;'
                    '小壮需4个苹果;小勇说和小壮一样;小康也和小壮一样;'
                    '小乐、小欣都需3个橘子。设数量。')
        result = demand_tool(question)
        self.assertEqual(result['totals'], [12,6])
        self.assertEqual((result['x'], result['y']), (3,2))

    def test_actual_duplicate_person_is_still_rejected(self):
        question = ('每个箱有5个玩具;有小车、积木两种玩具箱;'
                    '小杰需4个小车;小杰需1个积木。')
        with self.assertRaisesRegex(ValueError,'人物重复.*小杰'):
            demand_tool(question)


class MapTests(unittest.TestCase):
    def test_nonzero_origin_and_resolution_select_correct_pixels(self):
        image = np.arange(100,dtype=np.uint8).reshape(10,10)
        meta = {'resolution': .5, 'origin': [-2.,-3.,0.]}
        crop = crop_occupancy_map(image,meta,(-1.,1.,-2.,0.))
        np.testing.assert_array_equal(crop,image[3:8,2:7])

    def test_actual_v2_map_can_be_encoded(self):
        directory = ROOT/'ros_ws/src/yzbot/bot_navigation/maps'
        meta = yaml.safe_load((directory/'map.yaml').read_text())
        image = cv2.imread(str(directory/meta['image']),0)
        bounds = json.loads((ROOT/'ros_ws/scene_config.json').read_text())['search_bounds']
        crop = crop_occupancy_map(image,meta,bounds)
        self.assertGreater(crop.shape[0],300)
        self.assertGreater(crop.shape[1],500)
        self.assertTrue(cv2.imencode('.png',crop)[0])

    def test_outside_map_is_explicitly_rejected(self):
        with self.assertRaisesRegex(ValueError,'没有交集'):
            crop_occupancy_map(np.zeros((10,10),np.uint8),
                               {'resolution':1.,'origin':[0.,0.,0.]},(20.,30.,20.,30.))


if __name__ == '__main__':
    unittest.main()
