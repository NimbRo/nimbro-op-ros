//rqt plugin for tuning parameters
//Author: Sebastian Schüller

#include <pluginlib/class_list_macros.h>
#include <boost/iterator/iterator_concepts.hpp>

#include <config_server/Save.h>
#include <config_server/Load.h>

#include "parametertuner.h"
#include "parameteritem.h"
#include <config_server/parameterclient.h>

namespace parametertuner
{

Parametertuner::Parametertuner()
 : m_useJoystick(false)
{}

Parametertuner::~Parametertuner()
{
}


void Parametertuner::initPlugin(qt_gui_cpp::PluginContext& context)
{
	QWidget* w = new QWidget();
	m_ui.setupUi(w);

	connect(m_ui.joystick_button, SIGNAL(toggled(bool)), this, SLOT(handleJoystickButton()));
	connect(m_ui.save_button, SIGNAL(clicked(bool)), this, SLOT(save()));
	connect(m_ui.reset_button, SIGNAL(clicked(bool)), this, SLOT(reset()));

	m_sub_paramList = getNodeHandle().subscribe("/config_server/parameter_list", 1, &Parametertuner::handleList, this);
	m_sub_joystick = getNodeHandle().subscribe("/joy", 1, &Parametertuner::handleJoystickInput, this);
	connect(this, SIGNAL(updateRequested()), this, SLOT(update()), Qt::QueuedConnection);
	connect(this, SIGNAL(moveSelectionRequested(int)), this, SLOT(moveSelection(int)), Qt::QueuedConnection);
	connect(this, SIGNAL(ValueChangeRequested(int)), this, SLOT(ChangeValue(int)), Qt::QueuedConnection);

	m_ui.parameter_root_widget->setColumnCount(2);
	m_ui.parameter_root_widget->setColumnWidth(0, 200);

	context.addWidget(w);
}

void Parametertuner::handleList(const config_server::ParameterListConstPtr& list)
{
	QMutexLocker locker(&m_mutex);
	m_list = list;
	updateRequested();
}

void Parametertuner::deleteWidget(QTreeWidgetItem* item)
{
	QWidget* w = m_ui.parameter_root_widget->itemWidget(item, 1);
	m_ui.parameter_root_widget->removeItemWidget(item, 1);

	if(w)
		delete w;

	for(int i = 0; i < item->childCount(); ++i)
		deleteWidget(item->child(i));
}

void Parametertuner::update()
{
	deleteWidget(m_ui.parameter_root_widget->invisibleRootItem());
	m_ui.parameter_root_widget->clear();

	qApp->processEvents();

	config_server::ParameterClient* client = config_server::ParameterClient::instance();
	client->cork();

	QMutexLocker locker (&m_mutex);
	for(size_t i = 0; i < m_list->parameters.size(); i++)
	{
		QString param = QString::fromStdString(m_list->parameters[i].name);
		insertParameter(param, 0, m_list->parameters[i]);
	}

	client->uncork();
}


void Parametertuner::insertParameter(QString param, QTreeWidgetItem* ancestor, const config_server::ParameterDescription& description)
{
	if (param.isEmpty())
		return;

	QString levelParam, strippedParam;
	bool isLeaf;
	levelParam = param.section("/", 1, 1);
	strippedParam = param.section("/", 2);
	isLeaf = strippedParam.isEmpty();
	if (!isLeaf)
		strippedParam = "/" + strippedParam;


	QTreeWidgetItem* nextItem = 0;
	if (!ancestor)
	{
		QList<QTreeWidgetItem *> topLevelItems;
		topLevelItems = m_ui.parameter_root_widget->findItems(levelParam, 0);
		if (topLevelItems.isEmpty())
		{
			QTreeWidgetItem* newItem = 0;
			newItem = new QTreeWidgetItem();
			newItem->setText(0, levelParam);
			m_ui.parameter_root_widget->addTopLevelItem(newItem);
			nextItem  = newItem;
		}
		else
		{
			nextItem = topLevelItems.at(0);
		}
		insertParameter(strippedParam, nextItem, description);
		return;
	}

	int childCount = ancestor->childCount();
	int i;
	for (i = 0; i < childCount; i++)
	{
		if (levelParam == ancestor->child(i)->text(0))
		{
			insertParameter(strippedParam, ancestor->child(i), description);
			return;
		}
	}

	if (isLeaf)
		nextItem = insertParameterNode(description, ancestor);
	else
		nextItem = new QTreeWidgetItem(ancestor);

	nextItem->setText(0, levelParam);
	insertParameter(strippedParam, nextItem, description);
	return;

}

QTreeWidgetItem* Parametertuner::insertParameterNode(const config_server::ParameterDescription& description, QTreeWidgetItem* ancestor)
{
	QTreeWidgetItem* newItem = new QTreeWidgetItem(ancestor);

	QWidget* newWidget = 0;
	QString type = QString::fromStdString(description.type);
	if (type == "int")
		newWidget = new IntParameterWidget(getNodeHandle(), description);
	else if (type == "string")
		newWidget = new StringParameterWidget(getNodeHandle(), description);
	else if (type == "float")
		newWidget = new FloatParameterWidget(getNodeHandle(), description);
	else if (type == "bool")
		newWidget = new BoolParameterWidget(getNodeHandle(), description);

	if (newWidget != 0)
		m_ui.parameter_root_widget->setItemWidget(newItem, 1, newWidget);

	return newItem;
}

void Parametertuner::save()
{
	config_server::Save srv;

	if (!ros::service::call("/config_server/save",srv))
	{
		m_ui.save_button->setText("saving failed");
		ROS_WARN("Could not call config_server service 'save'!");
		return;
	}
	m_ui.save_button->setText("save");
}

void Parametertuner::reset()
{
	config_server::Load srv;

	srv.request.filename = "";

	if (!ros::service::call("/config_server/load", srv))
	{
		m_ui.reset_button->setText("failed");
		ROS_WARN("Could not call config_server service 'load'!");
	}
	m_ui.reset_button->setText("reset");
}


void Parametertuner::handleJoystickButton()
{
	m_useJoystick = m_ui.joystick_button->isChecked();
}

void Parametertuner::handleJoystickInput(const sensor_msgs::JoyConstPtr& joy)
{
	if (!m_useJoystick)
		return;

	int buttonUp = 4;
	int buttonDown = 6;
	int buttonLeft = 5;
	int buttonRight = 7;

	if (m_buttonUp && !joy->buttons[buttonUp])
		moveSelectionRequested(UP);
	m_buttonUp = joy->buttons[buttonUp];

	if (m_buttonDown && !joy->buttons[buttonDown])
		moveSelectionRequested(DOWN);
	m_buttonDown = joy->buttons[buttonDown];

	if (m_buttonDec && !joy->buttons[buttonLeft])
		ValueChangeRequested(LEFT);
	m_buttonDec = joy->buttons[buttonLeft];

	if (m_buttonInc && !joy->buttons[buttonRight])
		ValueChangeRequested(RIGHT);
	m_buttonInc = joy->buttons[buttonRight];

}

void Parametertuner::moveSelection(int dir)
{
	QTreeWidgetItem* thisItem = getSelectedItem();
	if (!thisItem)
	{
		m_ui.parameter_root_widget->topLevelItem(0)->setSelected(true);
		return;
	}
	QTreeWidgetItem* parent = thisItem->parent();
	QTreeWidgetItem* nextItem;

	if (!parent)
	{
		parent = m_ui.parameter_root_widget->invisibleRootItem();
	}
	int thisItemIndex = parent->indexOfChild(thisItem);
	if (thisItem->isExpanded() && dir == DOWN)
	{
		nextItem = thisItem->child(0);
		updateSelection(thisItem, nextItem);
		return;
	}
	else if (!parent->child(thisItemIndex + dir))
	{
		updateSelection(thisItem, parent);
		return;
	}
	else
	{
		nextItem = parent->child(thisItemIndex + dir);
		updateSelection(thisItem, nextItem);
		return;
	}

}

QTreeWidgetItem* Parametertuner::getSelectedItem()
{
	QList<QTreeWidgetItem *> selectList = m_ui.parameter_root_widget->selectedItems();
	if (selectList.size() == 0)
	{
		return 0;
	}
	else
		return selectList.at(0);
}

void Parametertuner::updateSelection(QTreeWidgetItem* old, QTreeWidgetItem* next)
{
	old->setSelected(false);
	next->setSelected(true);
	m_ui.parameter_root_widget->scrollToItem(next);
}

void Parametertuner::ChangeValue(int dir)
{
	QTreeWidgetItem* thisItem;
	if (!(thisItem = getSelectedItem()))
		return;

	if (thisItem->childCount() != 0)
	{
		if (dir == RIGHT && (!thisItem->isExpanded()))
			thisItem->setExpanded(true);
		else if (dir == LEFT && thisItem->isExpanded())
			thisItem->setExpanded(false);
		return;
	}
	ParameterWidgetBase* param;
	param = (ParameterWidgetBase *)m_ui.parameter_root_widget->itemWidget(thisItem, 1);
	if (!param)
		return;

	if (dir == LEFT)
		param->DecValue();
	else
		param->IncValue();
}


}

PLUGINLIB_EXPORT_CLASS(parametertuner::Parametertuner, rqt_gui_cpp::Plugin)
